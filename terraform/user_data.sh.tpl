#!/bin/bash
set -euo pipefail

# ─── System setup ────────────────────────────────────────────────────

apt-get update -y
apt-get install -y python3.11 python3.11-venv python3-pip git caddy

# ─── App setup ───────────────────────────────────────────────────────

APP_DIR=/opt/dimos-teleop
mkdir -p $APP_DIR
cd $APP_DIR

# Clone the repo (or copy from S3 — adjust as needed)
# For now, write the app inline from user_data
cat > .env << 'ENVEOF'
CF_TELEOP_APP_ID=${cf_teleop_app_id}
CF_TELEOP_APP_SECRET=${cf_teleop_app_secret}
JWT_SECRET=${jwt_secret}
DATABASE_URL=sqlite+aiosqlite:///./teleop.db
HOST=127.0.0.1
PORT=${app_port}
ENVEOF

chmod 600 .env

# ─── Python venv ─────────────────────────────────────────────────────

python3.11 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install \
  fastapi==0.115.12 \
  'uvicorn[standard]==0.34.3' \
  httpx==0.28.1 \
  pydantic==2.11.3 \
  pydantic-settings==2.9.1 \
  'python-jose[cryptography]==3.4.0' \
  'passlib[bcrypt]==1.7.4' \
  sqlalchemy==2.0.41 \
  aiosqlite==0.21.0 \
  python-multipart==0.0.20

# ─── Systemd service ────────────────────────────────────────────────

cat > /etc/systemd/system/dimos-teleop.service << 'SVCEOF'
[Unit]
Description=dimos-teleop session microservice
After=network.target

[Service]
Type=simple
WorkingDirectory=/opt/dimos-teleop/app
ExecStart=/opt/dimos-teleop/.venv/bin/uvicorn main:app --host 127.0.0.1 --port ${app_port}
Restart=always
RestartSec=5
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
SVCEOF

# ─── Caddy reverse proxy (HTTPS) ────────────────────────────────────

# Caddy auto-provisions TLS via Let's Encrypt.
# Until DNS is pointed, it serves on :80/:443 with self-signed.
cat > /etc/caddy/Caddyfile << 'CADDYEOF'
:80, :443 {
    reverse_proxy 127.0.0.1:${app_port}
}
CADDYEOF

# ─── Start services ─────────────────────────────────────────────────

systemctl daemon-reload
systemctl enable dimos-teleop
systemctl start dimos-teleop
systemctl restart caddy

echo "dimos-teleop deployed successfully"
