# Default command when 'just' is run without arguments
default:
  @just --list

# Lint files
[group('Main')]
lint:
  nix fmt && uv run ruff check --fix src

# Format files
[group('Main')]
fmt:
  nix fmt && uv run ruff format src

# Update nix flake
[group('Nix')]
update-nix:
  nix flake update

# Show flake outputs
[group('Nix')]
show-nix:
  nix flake show --all-systems --legacy

# Enter Nix REPL
[group('Nix')]
repl-nix:
  nix repl .

# Run typechecking
check-py:
  uv run basedpyright --project pyproject.toml

# Sync packages
[group('Python')]
sync-py:
  uv sync --all-packages

# Forcefully sync packages
[group('Python')]
sync-clean-py:
  uv sync --all-packages --force-reinstall --no-cache
