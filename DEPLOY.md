# Deployment Guide

## Prerequisites

- AWS CLI configured with full access (EC2, VPC, EIP, Route53)
- Terraform installed
- `daneel-local.pem` SSH key (must match AWS key pair `daneel-local` in us-east-2)
- GitHub repo access to `dimensionalOS/dimensional-teleop`

## Secrets

Set these as GitHub repository secrets (Settings → Secrets → Actions) for CI, or pass directly to Terraform:

| Secret | Description |
|--------|-------------|
| `CF_TELEOP_APP_ID` | Cloudflare Realtime SFU App ID |
| `CF_TELEOP_APP_SECRET` | Cloudflare Realtime SFU App Secret |
| `JWT_SECRET` | Random string for signing auth tokens (auto-generated if omitted) |

Find CF credentials in the Cloudflare dashboard: [Realtime SFU](https://dash.cloudflare.com/?to=/:account/realtime/sfu) → `hosted-teleop-dev-0` app.

## Step 1: Terraform — Provision EC2

```bash
git clone https://github.com/dimensionalOS/dimensional-teleop.git
cd dimensional-teleop/terraform
cp terraform.tfvars.example terraform.tfvars
```

Edit `terraform.tfvars` with your actual values:
```hcl
aws_region           = "us-east-2"
instance_type        = "t3.small"
key_name             = "daneel-local"
cf_teleop_app_id     = "<from CF dashboard>"
cf_teleop_app_secret = "<from CF dashboard>"
```

Deploy:
```bash
terraform init
terraform apply
```

Outputs:
- `public_ip` — Elastic IP (static, survives reboots)
- `ssh_command` — Ready-to-use SSH command
- `api_url` — HTTP endpoint for health check

## Step 2: DNS — Route53

Create an A record pointing `teleop.dimensionalos.com` to the Elastic IP.

**Option A: Manual (Route53 console)**
1. Go to Route53 → `dimensionalos.com` hosted zone
2. Create record:
   - Name: `teleop`
   - Type: `A`
   - Value: `<elastic_ip from terraform output>`
   - TTL: `300`

**Option B: Terraform (automated)**
1. Find your Route53 hosted zone ID for `dimensionalos.com`
2. Uncomment the block in `terraform/route53.tf`
3. Set `route53_zone_id` in your tfvars
4. `terraform apply`

## Step 3: Deploy App Code

SSH into the instance and clone the repo:

```bash
ssh -i daneel-local.pem ubuntu@<elastic_ip>

# The instance bootstraps Python, Caddy, and systemd on first boot.
# Clone the app code:
sudo mkdir -p /opt/dimos-teleop
sudo chown ubuntu:ubuntu /opt/dimos-teleop
cd /opt/dimos-teleop
git clone https://github.com/dimensionalOS/dimensional-teleop.git repo
ln -sf repo/app app

# Create .env with real credentials
cat > app/.env << 'EOF'
CF_TELEOP_APP_ID=<your-app-id>
CF_TELEOP_APP_SECRET=<your-app-secret>
JWT_SECRET=<random-string>
DATABASE_URL=sqlite+aiosqlite:///./teleop.db
HOST=127.0.0.1
PORT=8450
EOF
chmod 600 app/.env

# Install deps and start
cd /opt/dimos-teleop
python3.11 -m venv .venv || python3 -m venv .venv
source .venv/bin/activate
pip install -r app/requirements.txt

# Start the service
sudo systemctl restart dimos-teleop
```

**Or** use the deploy script from your local machine for subsequent updates:
```bash
./scripts/deploy.sh <elastic_ip>
```

## Step 4: Configure HTTPS (Caddy)

Once DNS propagates (`dig teleop.dimensionalos.com` returns the EIP), update Caddy:

```bash
ssh -i daneel-local.pem ubuntu@<elastic_ip>
sudo tee /etc/caddy/Caddyfile << 'EOF'
teleop.dimensionalos.com {
    reverse_proxy 127.0.0.1:8450
}
EOF
sudo systemctl restart caddy
```

Caddy auto-provisions Let's Encrypt TLS. HTTPS is live within seconds.

## Step 5: Verify

```bash
# Health check
curl https://teleop.dimensionalos.com/health
# → {"status":"ok","service":"dimos-teleop"}

# API docs (Swagger UI)
open https://teleop.dimensionalos.com/docs

# Register a test operator
curl -X POST https://teleop.dimensionalos.com/api/v1/auth/register \
  -H "Content-Type: application/json" \
  -d '{"email":"test@dimensional.io","password":"testpass"}'
```

## Architecture

```
This microservice handles ONLY:
  - Auth (login, register, robot API keys)
  - Session lifecycle (create, join, leave, list, heartbeat)
  - SDP exchange with Cloudflare Realtime SFU API

Real-time data (video, pose commands) flows DIRECTLY:
  Operator ←→ Cloudflare Edge (WebRTC) ←→ Robot
  This EC2 is NOT in the real-time path.
```

## Updating

Push to `main`, then:
```bash
./scripts/deploy.sh <elastic_ip>
```

Or SSH in and `git pull` + `sudo systemctl restart dimos-teleop`.
