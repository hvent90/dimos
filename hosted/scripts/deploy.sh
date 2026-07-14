#!/bin/bash
# Deploy code to an existing dimos-teleop EC2 instance.
# Usage: ./scripts/deploy.sh <ip-address>
#
# Run from the repo root. rsyncs app/ + web/ into /opt/dimos-teleop/ and the
# repo Caddyfile (single source of truth — do NOT hand-edit on the box) into
# /etc/caddy/, then restarts the units. Assumes user_data has already
# bootstrapped Python/Caddy/systemd and chowned /opt/dimos-teleop to ubuntu.

set -euo pipefail

IP="${1:?Usage: deploy.sh <ip-address>}"
KEY="${SSH_KEY:-daneel-local.pem}"
SSH_OPTS="-i $KEY -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR"

echo "Deploying to $IP..."

rsync -avz --delete \
  --exclude __pycache__ --exclude '*.pyc' --exclude .venv --exclude '*.db' --exclude '*.db-wal' --exclude '*.db-shm' --exclude '.*-litestream' --exclude .env \
  -e "ssh $SSH_OPTS" \
  app/ ubuntu@$IP:/opt/dimos-teleop/app/

# Static SPA — Caddy serves /opt/dimos-teleop/web directly.
rsync -avz --delete -e "ssh $SSH_OPTS" web/ ubuntu@$IP:/opt/dimos-teleop/web/

# Caddyfile from the repo; reload (not restart) keeps TLS conns alive.
rsync -avz -e "ssh $SSH_OPTS" Caddyfile ubuntu@$IP:/tmp/Caddyfile.deploy
ssh $SSH_OPTS ubuntu@$IP '
  if ! sudo cmp -s /tmp/Caddyfile.deploy /etc/caddy/Caddyfile; then
    sudo cp /tmp/Caddyfile.deploy /etc/caddy/Caddyfile
    sudo systemctl reload caddy
    echo "Caddyfile updated + reloaded"
  fi
  rm -f /tmp/Caddyfile.deploy
'

# Refresh deps in case requirements.txt changed
ssh $SSH_OPTS ubuntu@$IP \
  '/opt/dimos-teleop/.venv/bin/pip install --quiet -r /opt/dimos-teleop/app/requirements.txt'

ssh $SSH_OPTS ubuntu@$IP 'sudo systemctl restart dimos-teleop'
sleep 2

echo "--- service health (from inside the box) ---"
ssh $SSH_OPTS ubuntu@$IP '
  sudo systemctl is-active dimos-teleop
  curl -sf http://127.0.0.1:8450/health && echo
'
