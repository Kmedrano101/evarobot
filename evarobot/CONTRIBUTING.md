# Contributing to EvaRobot

Thank you for your interest in contributing to EvaRobot!

## Code of Conduct

Please be respectful and constructive in all interactions.

## How to Contribute

### Reporting Bugs

- Use GitHub Issues to report bugs
- Describe the bug clearly with steps to reproduce
- Include system information (OS, ROS2 version, etc.)

### Suggesting Features

- Open a GitHub Issue with the "enhancement" label
- Describe the feature and its use case
- Be open to discussion

### Pull Requests

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Test your changes thoroughly
5. Commit with clear messages (`git commit -m 'Add amazing feature'`)
6. Push to your branch (`git push origin feature/amazing-feature`)
7. Open a Pull Request

### Coding Standards

- Follow ROS2 naming conventions
- Use meaningful variable and function names
- Add comments for complex logic
- Run linting before committing:
  ```bash
  colcon test --packages-select evarobot_<package>
  ```

### Testing

- Add unit tests for new functionality
- Ensure existing tests pass
- Test in both simulation and (if possible) real hardware

## Development Workflow

### Building

```bash
# Desktop (full build)
colcon build --symlink-install

# Deployment (minimal build)
colcon build --symlink-install \
    --packages-skip evarobot_mapping evarobot_viz evarobot_planning \
                    evarobot_motion evarobot_perception \
                    evarobot_cpp_examples evarobot_py_examples
```

### Testing

```bash
colcon test --packages-select evarobot_<package>
colcon test-result --verbose
```

## Questions?

Feel free to open an issue for questions or reach out to the maintainer.
