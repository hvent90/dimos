variable "aws_region" {
  default = "us-east-2"
}

variable "instance_type" {
  default = "t3.small"
}

variable "key_name" {
  default = "daneel-local"
}

variable "vpc_id" {
  description = "VPC ID. Use default VPC or specify existing."
  default     = ""
}

variable "subnet_id" {
  description = "Subnet ID. Leave empty to use first available in VPC."
  default     = ""
}

variable "app_port" {
  default = 8450
}

variable "ssh_ingress_cidrs" {
  description = "CIDRs allowed to SSH (port 22). Default keeps the historic open behavior; NARROW THIS to your admin IPs (e.g. [\"203.0.113.7/32\"])."
  type        = list(string)
  default     = ["0.0.0.0/0"]
}

variable "cf_teleop_app_id" {
  description = "Cloudflare Realtime SFU App ID"
  sensitive   = true
}

variable "cf_teleop_app_secret" {
  description = "Cloudflare Realtime SFU App Secret"
  sensitive   = true
}

variable "cf_turn_key_id" {
  description = "Cloudflare TURN key ID (Realtime → TURN). Empty = STUN-only."
  sensitive   = true
  default     = ""
}

variable "cf_turn_api_token" {
  description = "Cloudflare TURN key API token. Empty = STUN-only."
  sensitive   = true
  default     = ""
}
