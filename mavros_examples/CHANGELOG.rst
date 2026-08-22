^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mavros_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.15.1 (2026-08-22)
-------------------

2.15.0 (2026-08-08)
-------------------
* build: bump cmake_minimum_required to 3.10
  CMake (4.0+, as shipped on Lyrical/Rolling) emits a deprecation
  warning for cmake_minimum_required < 3.10:
  CMake Deprecation Warning at CMakeLists.txt:1 (cmake_minimum_required):
  Compatibility with CMake < 3.10 will be removed from a future version
  of CMake.
  Raise the floor to 3.10 for all packages that were below it:
  - libmavconn, mavros, mavros_msgs, mavros_extras: 3.5 -> 3.10
  - test_mavros: 2.8.3 -> 3.10
  - mavros_examples: 3.8 -> 3.10
  3.10 is the lowest version that silences the warning while remaining
  compatible with all supported ROS 2 distros (Humble ships CMake 3.22).
* Merge pull request `#2242 <https://github.com/mavlink/mavros/issues/2242>`_ from mavlink/followup-terrain-protocol
  extras: polish terrain protocol integration (PR `#2137 <https://github.com/mavlink/mavros/issues/2137>`_ followup)
* common: modernize Python packaging to pyproject.toml for all packages
  - mavros: add pyproject.toml, simplify setup.py to shim, trim setup.cfg
  - mavros_examples: add pyproject.toml, simplify setup.py, trim setup.cfg,
  delete ruff.toml (merged into pyproject.toml)
  - mavros: ignore CNL100 flake8 rule (class def without blank line)
* Contributors: Vladimir Ermakov

2.14.0 (2025-12-23)
-------------------
* fix import order in px4 offboard script
* feat: add px4_offboard entry point to setup.cfg
* refactor: improve docstring for timer_callback method, adhering to pep257
* apply ruff to px4 offboard example
* lint: solve flake8 violations in px4 offboard example
* feat: add px4 offboard control example script
* Contributors: Davide Iafrate, Davide iafrate

2.13.0 (2025-12-12)
-------------------
* examples: disable copyright check
* examples: add copyright headers
* examples: fix package.xml lint
* examples: fix doc strings, add types
* examples: isort + ruff format
* examples: add sample setup.py
* update packages to format 3
* modifying package.xml file.
* setting mavros_examples for draft pr
* adding mavros_examples
* Contributors: Haroon Rasheed, Vladimir Ermakov, haroon
