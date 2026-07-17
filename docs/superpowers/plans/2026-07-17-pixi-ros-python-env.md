# pixi ROS (Humble/Jazzy) + Python Environment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Provision ROS 2 (Humble + Jazzy, Jazzy default) and Python/toolchain deps through pixi + RoboStack, fully replacing rosdep and Poetry.

**Architecture:** A `pixi.toml` with a conda-forge base (toolchain + Python deps) and one feature per ROS distro (its own `robostack-<distro>` channel + `ros-<distro>-*` packages). Two environments (`humble`, `jazzy`) plus `default = ["jazzy"]`. pixi tasks build/test/run each distro into a `$PIXI_ENVIRONMENT_NAME`-keyed base. CI, pre-commit, docs, and dependency metadata are migrated off rosdep/Poetry onto pixi.

**Tech Stack:** pixi 0.70.2, RoboStack (robostack-humble / robostack-jazzy), conda-forge, colcon/ament_cmake, ruff, pre-commit, GitHub Actions.

## Global Constraints

- Platforms: `linux-64` only.
- Default environment: `jazzy`. Supported distros: `humble`, `jazzy`.
- Python deps come from conda-forge; `pydantic` is NOT included (unused in code).
- ruff config stays in `pyproject.toml` (`target-version = "py310"`).
- No AI/editor attribution in commits/PRs (AGENTS.md). Conventional Commits.
- Do not bypass pre-commit hooks (no `--no-verify`).
- `package.xml` / `CMakeLists.txt` are retained (consumed by colcon).
- `pixi.toml`/`pixi.lock` version placeholder is `0.0.0` (release workflow rewrites it).

---

### Task 1: pixi manifest + lockfile (working build/test on both distros)

**Files:**

- Create: `pixi.toml`
- Create: `pixi.lock` (generated)
- Modify: `.gitignore` (add `.pixi/`)

**Interfaces:**

- Produces: pixi environments `default`(=jazzy), `humble`, `jazzy`; tasks `build`, `test`, `launch`, `lint`, `format`, `clean`, each run with `pixi run [-e <distro>] <task>`.

- [ ] **Step 1: Write `pixi.toml`**

```toml
[workspace]
name = "co-mlops-rosbag-metadata"
version = "0.0.0"
channels = ["conda-forge"]
platforms = ["linux-64"]

# Distro-agnostic toolchain + Python deps, inherited by every environment.
[dependencies]
colcon-common-extensions = "*"
cmake = "*"
c-compiler = "*"       # project(co_mlops_rosbag_metadata) enables C/CXX by default
cxx-compiler = "*"
pyyaml = "*"           # runtime: the node imports `yaml`
pytest = "*"           # test
ruff = "*"             # lint/format (config from pyproject.toml)
lark = "*"             # lets pytest load the launch_testing plugin

# Each distro builds into its own base keyed on $PIXI_ENVIRONMENT_NAME so
# switching -e <distro> never reuses another distro's colcon/CMake cache.
[tasks.build]
cmd = 'colcon build --packages-select co_mlops_rosbag_metadata --build-base "build/$PIXI_ENVIRONMENT_NAME" --install-base "install/$PIXI_ENVIRONMENT_NAME" --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON'

[tasks.test]
depends-on = ["build"]
cmd = 'colcon test --packages-select co_mlops_rosbag_metadata --build-base "build/$PIXI_ENVIRONMENT_NAME" --install-base "install/$PIXI_ENVIRONMENT_NAME" --event-handlers console_direct+ && colcon test-result --test-result-base "build/$PIXI_ENVIRONMENT_NAME" --verbose'

[tasks.launch]
args = [{ arg = "path" }]
depends-on = ["build"]
cmd = 'bash -c "source install/$PIXI_ENVIRONMENT_NAME/setup.bash && ros2 launch co_mlops_rosbag_metadata co_mlops_rosbag_metadata_publisher.launch.xml path:={{ path }}"'

[tasks.lint]
cmd = "ruff check --config pyproject.toml ."

[tasks.format]
cmd = "ruff format --config pyproject.toml ."

[tasks.clean]
cmd = "rm -rf build install log"

[feature.humble]
channels = [{ channel = "https://prefix.dev/robostack-humble", priority = 1 }]

[feature.humble.dependencies]
ros-humble-ros-base = "*"
ros-humble-rclpy = "*"
ros-humble-std-msgs = "*"
ros-humble-ament-cmake = "*"
ros-humble-ament-cmake-python = "*"
ros-humble-ament-cmake-pytest = "*"
ros-humble-launch = "*"
ros-humble-launch-ros = "*"
ros-humble-ament-index-python = "*"
ros-humble-launch-testing = "*"
ros-humble-launch-testing-ros = "*"
ros-humble-launch-testing-ament-cmake = "*"

[feature.jazzy]
channels = [{ channel = "https://prefix.dev/robostack-jazzy", priority = 1 }]

[feature.jazzy.dependencies]
ros-jazzy-ros-base = "*"
ros-jazzy-rclpy = "*"
ros-jazzy-std-msgs = "*"
ros-jazzy-ament-cmake = "*"
ros-jazzy-ament-cmake-python = "*"
ros-jazzy-ament-cmake-pytest = "*"
ros-jazzy-launch = "*"
ros-jazzy-launch-ros = "*"
ros-jazzy-ament-index-python = "*"
ros-jazzy-launch-testing = "*"
ros-jazzy-launch-testing-ros = "*"
ros-jazzy-launch-testing-ament-cmake = "*"

[environments]
default = ["jazzy"]
humble = ["humble"]
jazzy = ["jazzy"]
```

- [ ] **Step 2: Add `.pixi/` to `.gitignore`**

Append a `.pixi/` line (and confirm `build/`, `install/`, `log/` are already ignored; add any that are missing).

- [ ] **Step 3: Solve + generate the lockfile for both distros**

Run (may take several minutes; run in background if it exceeds the shell timeout):

```bash
pixi install -e jazzy
pixi install -e humble
```

Expected: both solve and download successfully; `pixi.lock` is created/updated.
If a `ros-<distro>-<pkg>` name fails to solve, search the channel
(`pixi search -c https://prefix.dev/robostack-<distro> 'ros-<distro>-*'`) and
correct the name, then re-run.

- [ ] **Step 4: Verify jazzy build + test**

```bash
pixi run -e jazzy build
pixi run -e jazzy test
```

Expected: build succeeds; `colcon test-result --verbose` reports the launch test
and the invalid-YAML pytest passing (0 failures).

- [ ] **Step 5: Verify humble build + test**

```bash
pixi run -e humble build
pixi run -e humble test
```

Expected: same as jazzy, into `build/humble` / `install/humble`.

- [ ] **Step 6: Verify default env resolves to jazzy and the launch task runs**

```bash
pixi run build            # no -e ⇒ default (jazzy)
timeout 8 pixi run -e jazzy launch path:=src/co_mlops_rosbag_metadata/test/fixtures/sample_metadata.yaml || true
```

Expected: `pixi run build` builds the jazzy base; the launch prints
`Published ... to /metadata` before the timeout kills it.

- [ ] **Step 7: Commit**

```bash
git add pixi.toml pixi.lock .gitignore
git commit -m "feat(pixi): add pixi manifest and lockfile for ROS Humble/Jazzy"
```

---

### Task 2: Remove Poetry from Python packaging

**Files:**

- Modify: `pyproject.toml` (drop `[tool.poetry]*` and `[build-system]`; keep `[tool.ruff]*`)
- Delete: `poetry.lock`

**Interfaces:**

- Consumes: pixi `lint`/`format` tasks from Task 1.
- Produces: a `pyproject.toml` containing only `[tool.ruff]*`.

- [ ] **Step 1: Rewrite `pyproject.toml` to keep only the ruff config**

New full file content:

```toml
[tool.ruff]
target-version = "py310"
line-length = 100
src = ["src"]

[tool.ruff.lint]
select = [
    "E",      # pycodestyle errors
    "W",      # pycodestyle warnings
    "F",      # Pyflakes
    "I",      # isort
    "B",      # flake8-bugbear
    "C4",     # flake8-comprehensions
    "UP",     # pyupgrade
]
ignore = [
    "E501",   # line too long (handled by formatter)
]

[tool.ruff.lint.isort]
known-first-party = ["co_mlops_rosbag_metadata"]
split-on-trailing-comma = true
lines-after-imports = 2

[tool.ruff.format]
quote-style = "double"
docstring-code-format = true
```

- [ ] **Step 2: Delete the Poetry lockfile**

```bash
git rm poetry.lock
```

- [ ] **Step 3: Verify ruff still runs via pixi with the retained config**

```bash
pixi run -e jazzy lint
pixi run -e jazzy format
```

Expected: `ruff check` reports "All checks passed!" and `ruff format` reports the
files are already formatted (no changes).

- [ ] **Step 4: Commit**

```bash
git add pyproject.toml
git commit -m "refactor(deps): drop Poetry; keep ruff config in pyproject.toml"
```

---

### Task 3: Migrate pre-commit ruff hooks + pre-commit workflow off Poetry

**Files:**

- Modify: `.pre-commit-config.yaml` (replace local `poetry run ruff` hooks)
- Modify: `.github/workflows/pre-commit.yaml` (remove the Poetry install step)

**Interfaces:**

- Consumes: `[tool.ruff]` in `pyproject.toml` (Task 2).

- [ ] **Step 1: Replace the local ruff hooks with the official ruff-pre-commit**

In `.pre-commit-config.yaml`, delete the entire `- repo: local` block (the
`ruff-check` and `ruff-format` `poetry run` hooks) and insert this block in the
same position (after the `tier4/pre-commit-hooks-ros` block):

```yaml
- repo: https://github.com/astral-sh/ruff-pre-commit
  rev: v0.8.6
  hooks:
    - id: ruff
      args: [--fix]
    - id: ruff-format
```

- [ ] **Step 2: Remove the Poetry step from the pre-commit workflow**

In `.github/workflows/pre-commit.yaml`, delete the step:

```yaml
- name: Install Poetry and dependencies
  run: |
    pip install poetry
    poetry install
```

Leave the Checkout, Set up Python, Set up Node, Install pre-commit, and Run
pre-commit steps unchanged.

- [ ] **Step 3: Verify pre-commit passes end-to-end**

```bash
pre-commit run --all-files -c .pre-commit-config.yaml
```

Expected: all hooks pass (ruff now provided by the official hook; no Poetry
needed). If ruff-format reformats anything, re-stage and re-run until clean.

- [ ] **Step 4: Commit**

```bash
git add .pre-commit-config.yaml .github/workflows/pre-commit.yaml
git commit -m "ci(pre-commit): use official ruff-pre-commit; drop Poetry from workflow"
```

---

### Task 4: Migrate CI/release workflows to pixi; remove rosdep

**Files:**

- Delete: `rosdep.yaml`
- Delete: `.github/workflows/test.yaml`
- Create: `.github/workflows/build.yaml`
- Modify: `.github/workflows/release.yaml`

**Interfaces:**

- Consumes: pixi environments/tasks from Task 1.

- [ ] **Step 1: Remove rosdep and the old container test workflow**

```bash
git rm rosdep.yaml .github/workflows/test.yaml
```

- [ ] **Step 2: Create `.github/workflows/build.yaml`**

```yaml
name: build

on:
  pull_request:
  push:
    branches:
      - main

jobs:
  pixi:
    strategy:
      fail-fast: false
      matrix:
        distro: [humble, jazzy]
    runs-on: ubuntu-latest
    steps:
      - name: Checkout
        uses: actions/checkout@v4

      # pixi provisions ROS 2 (RoboStack) + the toolchain (conda-forge) from the
      # committed pixi.lock, so no system ROS install or rosdep is needed.
      - name: Set up pixi
        uses: prefix-dev/setup-pixi@v0.9.6
        with:
          pixi-version: v0.70.2
          environments: ${{ matrix.distro }}

      - name: Build
        run: pixi run -e ${{ matrix.distro }} build

      - name: Test
        run: pixi run -e ${{ matrix.distro }} test
```

- [ ] **Step 3: Rewrite `.github/workflows/release.yaml`**

Keep the `release` job's tag-parse / branch / tag-move logic, but (a) rewrite the
version-replacement to target `pixi.toml` instead of `pyproject.toml`, and
(b) replace the container-based `build` job with a pixi matrix. Full new file:

```yaml
name: release

on:
  push:
    tags:
      - v*-*

jobs:
  release:
    runs-on: ubuntu-22.04
    permissions:
      contents: write
    steps:
      - name: Checkout tag
        uses: actions/checkout@v4
        with:
          ref: ${{ github.ref }}
          token: ${{ secrets.GITHUB_TOKEN }}

      - name: Parse tag and set package version
        id: version
        run: |
          TAG="${GITHUB_REF#refs/tags/}"
          # Tag format: vX.X.X-Y.Y.Y (X = package version, Y = schema version)
          REST="${TAG#v}"
          PKG="${REST%-*}"
          SCHEMA="${REST#*-}"
          VERSION="${PKG}-${SCHEMA}"
          echo "version=${VERSION}" >> "$GITHUB_OUTPUT"
          echo "tag=${TAG}" >> "$GITHUB_OUTPUT"

      - name: Skip if version already replaced
        id: skip
        run: |
          if grep -q '<version>0.0.0</version>' src/co_mlops_rosbag_metadata/package.xml; then
            echo "replace=yes" >> "$GITHUB_OUTPUT"
          else
            echo "replace=no" >> "$GITHUB_OUTPUT"
          fi

      - name: Create release branch and commit version replacement
        if: steps.skip.outputs.replace == 'yes'
        run: |
          VERSION="${{ steps.version.outputs.version }}"
          TAG="${{ steps.version.outputs.tag }}"
          BRANCH="release/${TAG}"
          git config user.name "github-actions[bot]"
          git config user.email "github-actions[bot]@users.noreply.github.com"
          git checkout -b "${BRANCH}"
          sed -i "s|<version>0.0.0</version>|<version>${VERSION}</version>|g" \
            src/co_mlops_rosbag_metadata/package.xml
          sed -i 's/^version = "0.0.0"/version = "'"${VERSION}"'"/' pixi.toml
          git add src/co_mlops_rosbag_metadata/package.xml pixi.toml
          git commit -m "chore: set version to ${VERSION} for release"
          git push origin "${BRANCH}"

      - name: Move tag to version-replacement commit
        if: steps.skip.outputs.replace == 'yes'
        run: |
          TAG="${{ steps.version.outputs.tag }}"
          git push origin ":refs/tags/${TAG}"
          git tag -d "${TAG}" 2>/dev/null || true
          git tag "${TAG}"
          git push origin "${TAG}"

  build:
    runs-on: ubuntu-latest
    needs: release
    strategy:
      fail-fast: false
      matrix:
        distro: [humble, jazzy]
    steps:
      - uses: actions/checkout@v4
        with:
          ref: refs/tags/${{ github.ref_name }}

      - name: Set up pixi
        uses: prefix-dev/setup-pixi@v0.9.6
        with:
          pixi-version: v0.70.2
          environments: ${{ matrix.distro }}

      - name: Build
        run: pixi run -e ${{ matrix.distro }} build

      - name: Test
        run: pixi run -e ${{ matrix.distro }} test
```

- [ ] **Step 4: Validate the workflow YAML with pre-commit**

```bash
pre-commit run --files .github/workflows/build.yaml .github/workflows/release.yaml -c .pre-commit-config.yaml
```

Expected: check-yaml / yamllint / prettier pass (re-stage if prettier reformats).

- [ ] **Step 5: Commit**

```bash
git add .github/workflows/build.yaml .github/workflows/release.yaml
git commit -m "ci: build/release via pixi matrix (humble, jazzy); remove rosdep and container CI"
```

---

### Task 5: Update docs (README, AGENTS) for pixi

**Files:**

- Modify: `README.md` (Installation / Build / Testing / Usage)
- Modify: `AGENTS.md` (dependency convention)

**Interfaces:**

- Consumes: pixi tasks/environments from Task 1.

- [ ] **Step 1: Replace the README "Installation / Build / Testing / Usage" sections**

Replace the current sections (from `### Installation` through the end of the
`#### Example` block) with:

````markdown
### Prerequisites

Install [pixi](https://pixi.sh) (no system ROS 2 install is required — pixi
provisions ROS 2 from RoboStack and the toolchain from conda-forge):

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

### Environments

Two ROS 2 distributions are available as pixi environments; `jazzy` is the
default.

- `jazzy` (default)
- `humble`

### Build

```bash
pixi run build            # default (jazzy)
pixi run -e humble build  # Humble
```

### Testing

```bash
pixi run test             # default (jazzy)
pixi run -e humble test   # Humble
```

### Usage

`launch` builds (if needed) and starts the publisher node. Pass the YAML path
with `path:=`:

```bash
pixi run launch path:=/path/to/config.yaml
```

Or open an interactive shell with ROS 2 on `PATH`:

```bash
pixi shell -e jazzy
ros2 launch co_mlops_rosbag_metadata co_mlops_rosbag_metadata_publisher.launch.xml \
  path:=/path/to/config.yaml
```

#### Parameters

- `path` (required): Path to the YAML file to load.
- `topic` (default `/metadata`): Topic name.
- `delay_before_first_publish` (default `0.0`): Delay (s) before the first publish; 0 or less = best effort.
- `frequency` (default `1.0`): Republish rate in Hz; 0 for one-shot.
````

- [ ] **Step 2: Update the AGENTS.md dependency rule**

Replace the `- **Dependencies**: ...` bullet under "General rules" with:

```markdown
- **Dependencies**: This repository uses **pixi** (RoboStack for ROS 2 + conda-forge for the toolchain and Python packages) to provision the environment. Add ROS and Python dependencies to `pixi.toml` and commit the updated `pixi.lock`. `package.xml` is retained as ament build metadata consumed by colcon; do not reintroduce rosdep or Poetry.
```

- [ ] **Step 3: Verify docs pass pre-commit**

```bash
pre-commit run --files README.md AGENTS.md -c .pre-commit-config.yaml
```

Expected: markdownlint / prettier / markdown-link-check pass (re-stage if
prettier reformats).

- [ ] **Step 4: Commit**

```bash
git add README.md AGENTS.md
git commit -m "docs: document pixi-based ROS Humble/Jazzy environment"
```

---

## Final verification

- [ ] `pixi run -e jazzy test` and `pixi run -e humble test` both pass.
- [ ] `pre-commit run --all-files -c .pre-commit-config.yaml` passes.
- [ ] `git grep -n rosdep` and `git grep -n poetry` return only historical/spec references (no active config).
- [ ] Open a PR to `main`; the `build` (humble/jazzy) and `pre-commit` checks are green.

## Notes / risks

- First `pixi install` downloads sizable conda packages; run long steps in the
  background if they exceed the shell timeout.
- RoboStack package names must match per distro; fix any that fail to solve.
- Renaming `test.yaml` -> `build.yaml` changes the required status-check name; a
  maintainer must update branch protection referencing `test`.
- The official ruff-pre-commit pin (`v0.8.6`) and the pixi `ruff` may drift in
  version; pin pixi `ruff` to the same series if formatting disagreements appear.
