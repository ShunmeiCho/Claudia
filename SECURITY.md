# Security Policy

## Supported Versions

| Version | Supported |
|---------|:---------:|
| main (latest) | Yes |
| Older commits | No |

## Reporting a Vulnerability

**Please do NOT report security vulnerabilities through public GitHub issues.**

### How to Report

Report security vulnerabilities through
[GitHub Security Advisories](https://github.com/nvidiasamp/Claudia/security/advisories/new).

Alternatively, send an email to **dev@claudia-robot.com** with:

- Description of the vulnerability
- Steps to reproduce
- Potential impact assessment
- Suggested fix (if any)

### What to Expect

| Timeline | Action |
|----------|--------|
| 48 hours | Acknowledgment of your report |
| 7 days | Initial assessment and severity classification |
| 30 days | Fix developed and tested (for confirmed vulnerabilities) |
| Post-fix | Public disclosure with credit (unless you prefer anonymity) |

## Security Considerations

Claudia controls a **physical robot**. Security issues in this project can have
real-world physical consequences. We take all reports seriously.

### Physical Safety (Critical)

The following components are critical to physical safety:

| Component | File | Risk |
|-----------|------|------|
| SafetyCompiler | `src/claudia/brain/safety_compiler.py` | Bypassing could cause uncontrolled robot movement |
| Battery Gating | `src/claudia/brain/safety_compiler.py` | Disabling could damage battery or cause unsafe behavior at low charge |
| Emergency Stop | `src/claudia/brain/production_brain.py` | Failure could prevent emergency shutdown |
| Action Registry | `src/claudia/brain/action_registry.py` | Incorrect flags could allow unsafe actions without prerequisites |
| Standing Prerequisites | `src/claudia/brain/safety_compiler.py` | Skipping could cause falls or collisions |

### Network Security

| Attack Surface | Details |
|---------------|---------|
| DDS Communication | Robot communicates via CycloneDDS on `192.168.123.x` network. No authentication by default |
| Ollama API | Local LLM API exposed on `localhost:11434`. Not authenticated |
| Unix Domain Sockets | ASR pipeline uses UDS (`/tmp/claudia_audio.sock`, `/tmp/claudia_asr_result.sock`) |

### Voice Input Security

| Concern | Details |
|---------|---------|
| Microphone Access | USB microphone captures audio continuously when voice mode is active |
| ASR Injection | Adversarial audio inputs could potentially trigger unintended robot actions |
| Wake Word Bypass | Emergency commands bypass wake word gating by design (this is intentional for safety) |

### LLM Security

| Concern | Details |
|---------|---------|
| Prompt Injection | Malicious voice/text input could attempt to manipulate LLM output |
| Action Code Injection | LLM output is validated against `VALID_API_CODES` whitelist before execution |
| Model Integrity | Local Ollama models should be verified against known checksums |

## Security Design Principles

1. **SafetyCompiler is the single gate**: All action execution paths go through
   `SafetyCompiler.compile()`. There is no bypass, regardless of input source.

2. **Whitelist, not blacklist**: Only actions registered in `action_registry.py`
   with `enabled=True` can execute. Unknown API codes are rejected.

3. **Battery gating is non-negotiable**: The 3-tier battery system cannot be
   disabled through configuration or environment variables.

4. **Emergency stop is hardcoded**: Emergency commands are pattern-matched
   before any LLM processing and cannot be overridden.

5. **High-risk actions are opt-in**: FrontFlip, FrontJump, and FrontPounce are
   disabled by default and require explicit `allow_high_risk=True`.

## Out of Scope

The following are known limitations, not vulnerabilities:

- **DDS network is trusted**: The `192.168.123.x` Ethernet link between Jetson
  and Go2 is assumed to be a private, physically secure connection
- **Local Ollama access**: The LLM API on `localhost:11434` is only accessible
  from the Jetson device itself
- **Physical access**: If an attacker has physical access to the Jetson or
  robot, all software protections can be bypassed

## Acknowledgments

We appreciate security researchers who help keep Claudia and its users safe.
Confirmed vulnerabilities will be acknowledged in release notes (with your
permission).
