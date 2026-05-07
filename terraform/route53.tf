# ─── Route53 DNS (Optional) ──────────────────────────────────────────
#
# Uncomment this block if you want Terraform to manage the DNS record.
# Otherwise, create the A record manually in Route53.
#
# Prerequisites:
#   1. dimensionalos.com hosted zone must exist in Route53
#   2. Set the zone_id below (find it in Route53 console)

# variable "route53_zone_id" {
#   description = "Route53 hosted zone ID for dimensionalos.com"
#   default     = ""
# }
#
# resource "aws_route53_record" "teleop" {
#   count   = var.route53_zone_id != "" ? 1 : 0
#   zone_id = var.route53_zone_id
#   name    = "teleop.dimensionalos.com"
#   type    = "A"
#   ttl     = 300
#   records = [aws_eip.teleop.public_ip]
# }
