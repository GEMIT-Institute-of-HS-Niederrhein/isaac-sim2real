# Contributing to Isaac Sim Dynamixel Bridge

Thank you for your interest in contributing! This document provides guidelines for contributing to this project.

## Getting Started

1. **Fork the repository** on GitHub
2. **Clone your fork** locally:
   ```bash
   git clone https://github.com/YOUR_USERNAME/IsaacSim.git
   cd IsaacSim/Prototype
   ```

3. **Set up the development environment**:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   ```

## Development Workflow

1. **Create a feature branch**:
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. **Make your changes** following our coding standards:
   - Use descriptive variable names
   - Add docstrings to functions and classes
   - Follow PEP 8 style guidelines
   - Add comments for complex logic

3. **Test your changes**:
   ```bash
   # Run verification script
   cd ~/Desktop/isaacsim/_build/linux-x86_64/release
   ./python.sh ~/Desktop/Prototype/scripts/verify_setup.py
   
   # Run tests
   source ~/Desktop/Prototype/.venv/bin/activate
   pytest tests/
   ```

4. **Commit your changes**:
   ```bash
   git add .
   git commit -m "feat: add your feature description"
   ```
   
   Use conventional commit messages:
   - `feat:` for new features
   - `fix:` for bug fixes
   - `docs:` for documentation changes
   - `test:` for test additions/changes
   - `refactor:` for code refactoring

5. **Push to your fork**:
   ```bash
   git push origin feature/your-feature-name
   ```

6. **Submit a Pull Request** on GitHub

## Code Style

- Python code should follow PEP 8
- Use `black` for formatting (optional but recommended)
- Maximum line length: 100 characters
- Use type hints where appropriate

## Testing

- Add tests for new features
- Ensure all existing tests pass
- Test with actual hardware when possible
- Document hardware requirements in PR

## Hardware Testing

If you don't have access to Dynamixel motors:
- Test GUI components independently using `simple_gui_test.py`
- Test Isaac Sim integration using `test_isaac_only.py`
- Clearly mark PRs as "untested on hardware"

## Documentation

- Update README.md if adding new features
- Add docstrings to new functions/classes
- Update config examples if changing configuration options
- Add troubleshooting tips for common issues

## Questions?

- Open an issue for bugs or feature requests
- Tag maintainers for questions
- Check existing issues before creating new ones

## Code of Conduct

- Be respectful and inclusive
- Help others learn
- Accept constructive feedback gracefully
- Focus on what's best for the project

Thank you for contributing! 🎉
