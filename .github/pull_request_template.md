## Description

<!-- A clear and concise description of what this PR does. -->

## Type of Change

<!-- Check all that apply. -->

- [ ] Bug fix (non-breaking change that fixes an issue)
- [ ] New feature (non-breaking change that adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to change)
- [ ] Refactoring (no functional changes)
- [ ] Documentation update
- [ ] Test addition / improvement
- [ ] CI / build configuration

## Related Issues

<!-- Link related issues. Use "Closes #123" to auto-close. -->

- Closes #
- Related to #

## Testing

### Simulation Testing

- [ ] Unit tests pass: `python3 test/run_tests.py --type unit`
- [ ] Integration tests pass: `python3 test/run_tests.py --type integration`
- [ ] Tested in keyboard + simulation mode
- [ ] Tested in voice + simulation mode (if voice pipeline affected)

### Hardware Testing

<!-- Check if applicable. Hardware tests are run by maintainers before merge. -->

- [ ] Not applicable (no hardware impact)
- [ ] Tested on physical Go2 robot
- [ ] Hardware testing requested (maintainer will verify)

## Safety Checklist

<!-- All PRs must confirm safety compliance. -->

- [ ] SafetyCompiler is not bypassed or weakened
- [ ] Battery gating thresholds are unchanged (or change is explicitly approved)
- [ ] Emergency stop behavior is not affected
- [ ] New actions are registered in `action_registry.py` with correct flags
- [ ] Standing prerequisites are correctly declared for new actions
- [ ] High-risk actions remain disabled by default

## Code Quality

- [ ] Code follows project style (Black formatted, line-length 88)
- [ ] Type hints added for new functions
- [ ] Python 3.8 compatible (no 3.9+ features)
- [ ] No hardcoded secrets or credentials
- [ ] No unnecessary `console.log` / `print` statements added

## Additional Notes

<!-- Any additional context, screenshots, or notes for reviewers. -->
