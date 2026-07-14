# ─── SES: branded sender for Cognito verification emails ─────────────
#
# Cognito's default sender (no-reply@verificationemail.com) lands in spam
# and caps at 50 emails/day. This verifies dimensionalos.com in SES with
# DKIM + custom MAIL FROM (SPF-aligned), so mail comes from
# no-reply@dimensionalos.com signed by our own domain.
#
# SES accounts start in sandbox (can only send to verified recipients).
# Production access is requested out-of-band (sesv2 put-account-details);
# flip `ses_email_enabled = true` and re-apply once AWS approves to switch
# the user pool from COGNITO_DEFAULT to the SES sender.

variable "ses_email_enabled" {
  description = "Use SES (no-reply@dimensionalos.com) for Cognito email. Only enable after SES production access is granted."
  type        = bool
  default     = true
}

data "aws_route53_zone" "main" {
  name = "dimensionalos.com"
}

resource "aws_ses_domain_identity" "main" {
  domain = "dimensionalos.com"
}

# TXT record proving domain ownership to SES.
resource "aws_route53_record" "ses_verification" {
  zone_id = data.aws_route53_zone.main.zone_id
  name    = "_amazonses.dimensionalos.com"
  type    = "TXT"
  ttl     = 600
  records = [aws_ses_domain_identity.main.verification_token]
}

resource "aws_ses_domain_identity_verification" "main" {
  domain     = aws_ses_domain_identity.main.id
  depends_on = [aws_route53_record.ses_verification]
}

# DKIM — the part that actually keeps mail out of spam.
resource "aws_ses_domain_dkim" "main" {
  domain = aws_ses_domain_identity.main.domain
}

resource "aws_route53_record" "ses_dkim" {
  count   = 3
  zone_id = data.aws_route53_zone.main.zone_id
  name    = "${aws_ses_domain_dkim.main.dkim_tokens[count.index]}._domainkey.dimensionalos.com"
  type    = "CNAME"
  ttl     = 600
  records = ["${aws_ses_domain_dkim.main.dkim_tokens[count.index]}.dkim.amazonses.com"]
}

# Custom MAIL FROM so the envelope sender aligns with our domain (SPF).
resource "aws_ses_domain_mail_from" "main" {
  domain           = aws_ses_domain_identity.main.domain
  mail_from_domain = "mail.dimensionalos.com"
}

resource "aws_route53_record" "ses_mail_from_mx" {
  zone_id = data.aws_route53_zone.main.zone_id
  name    = aws_ses_domain_mail_from.main.mail_from_domain
  type    = "MX"
  ttl     = 600
  records = ["10 feedback-smtp.${var.aws_region}.amazonses.com"]
}

resource "aws_route53_record" "ses_mail_from_spf" {
  zone_id = data.aws_route53_zone.main.zone_id
  name    = aws_ses_domain_mail_from.main.mail_from_domain
  type    = "TXT"
  ttl     = 600
  records = ["v=spf1 include:amazonses.com ~all"]
}

# Cognito needs explicit permission to send via this identity.
resource "aws_ses_identity_policy" "cognito_send" {
  identity = aws_ses_domain_identity.main.arn
  name     = "cognito-teleop-send"

  policy = jsonencode({
    Version = "2012-10-17"
    Statement = [{
      Sid       = "AllowCognitoSend"
      Effect    = "Allow"
      Principal = { Service = "cognito-idp.amazonaws.com" }
      Action    = ["ses:SendEmail", "ses:SendRawEmail"]
      Resource  = aws_ses_domain_identity.main.arn
      Condition = {
        StringEquals = { "aws:SourceAccount" = data.aws_caller_identity.current.account_id }
        ArnLike      = { "aws:SourceArn" = aws_cognito_user_pool.teleop.arn }
      }
    }]
  })
}

data "aws_caller_identity" "current" {}
