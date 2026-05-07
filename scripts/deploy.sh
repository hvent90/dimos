#!/bin/bash
# Deploy app code to an existing dimos-teleop EC2 instance.
# Usage: ./scripts/deploy.sh <ip-address>

set -euo pipefail

IP="${1:?Usage: deploy.sh <ip-address>}"
KEY="${SSH_KEY:-daneel-local.pem}"
SSH="ssh -i $KEY -o StrictHostKeyChecking=no ubuntu@$IP"
SCP="scp -i $KEY -o StrictHostKeyChecking=no"

echo "Deploying to $IP..."

# Sync app code
rsync -avz --delete \
  -e "ssh -i $KEY -o StrictHostKeyChecking=no" \
  app/ ubuntu@$IP:/opt/dimos-teleop/app/

# Restart service
$SSH "sudo systemctl restart dimos-teleop"

echo "Deployed. Health check:"
sleep 2
curl -s "http://$IP:8450/health" || echo "(might need port 443 via Caddy)"
