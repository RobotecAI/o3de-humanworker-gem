[![Apache License, Version 2.0][apache_shield]][apache]

# HumanWorker Gem for Open 3D Engine (O3DE)

## A bit of context

* [Open 3D Engine](https:://o3de.org) - an open-source game & simulation engine. O3DE is extendable through modules called Gems. This is one of such Gems.
* [Robot Operating System (ROS)](https://docs.ros.org/en/rolling/index.html) - an open-source middleware and, de facto, standard for robotics.
* [ROS2 Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/ROS2) - an open-source module for O3DE that enables simulation for robotics using modern ROS.

Please refer to [O3DE documentation](https://docs.o3de.org/docs/user-guide/gems/) to learn more about Gems and about registering Gems in the system and O3DE projects.

## Requirements
- Any O3DE project with the [O3DE ROS2 Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/ROS2) enabled.

## Description
This gem contains animated human worker assets that can be easily imported into an O3DE project. The human worker can navigate between the waypoints in the scene and is visible to any robots that maneuver in the same area.

In particular, this gem contains the following components:
- Human worker mesh with textures
- Mesh animations of human standing
- Mesh animations of human walking
- O3DE components for basic navigation of the human worker in a scene

Human worker objects are delivered as O3DE _prefabs_, containing visual models and physics, along with the required O3DE components. Four prefabs are provided within the gem. The simplest one, `HumanWorkerStatic.prefab`, is a _prefab_ combining mesh and textures. It can be used as a decoration in the scene. `HumanWorker.prefab` is additionally extended by animations. It does not contain any navigation-related components, therefore only _idle_ animation is used in this case. `HumanWorkerRobot.prefab` is a _prefab_ that can be used as a base for the fully functional non-player character (NPC) after adding the description of the scene (waypoints, etc.). `HumanWorkerNavigation.prefab` is an extension with placeholders for such descriptions that should be used in projects. 

![](docs/images/gem.png)

### How to use the Gem: NavigationExample
![](docs/images/navigation_example.png)

`NavigationExample.prefab` _prefab_ contains all the necessities to run the NPC in the scene. In particular, it consists of the animated mesh with textures, and the scene description in the format used by NPC navigation components. The scene description is added to `HumanWorkerNavigation.prefab` using O3DE _prefab overrides_.

![](docs/images/navigation_example_prefab.png)

- `HumanWorkerRobot` is a human worker prefab with some navigational components included. The _prefab_ is modified with the _overrides_ to correctly link with the scene description.
- `Endpoints` is a scene descriptor consisting of the ground truth positions used to build the path for the NPC. Two points are included in this example, but the number of points can be increased. All points need to be added to the `HumanWorkerRobot` _prefab_ `Npc Navigator` component to correctly build the path.
- `HumanWorkerNavigationMesh` is a scene descriptor component presenting the mesh where the NPC can move based on its movement capabilities and the scene itself. Please check [Recast framework documentation](https://recastnav.com) for more details. Tunable parameters include the size of the bounding box for mesh calculation and NPC characteristics. Please note, that all `Endpoints` need to be within the bounding box.
- `DetourNavigation` is a scene descriptor component for finding the path between the `Endpoints` within the navigation mesh.

## Release notes
### 2.0.0 for O3DE 2409.x
Changes compared to `1.1.0`
- Modified to work with `ROS2 Gem >= 3.1.0`
- Added _ScriptCanvas_ and _LUA_ support
- Added safety bubble (to stop NPC when inside a tagged space)
- Modified `NpcNavigatorRequestBus` to include `GoToLocation`
- Changed the namespace from `ROS2::HumanWorker` to `HumanWorker`
- Fixed prefabs
- Fixed build (switched off non-existing tests; modified cmake)

### 1.1.0 for O3DE 2310.x
Changes compared to `1.0.0`
- Added `NpcNavigatorNotificationBus` to notify about reaching the waypoint, refactor `NpcNavigatorRequestBus` to pass multiple waypoints at once
- Fixed prefabs
- Improved materials

### 1.0.0 for O3DE 2310.x
Initial release prepared for initial version of [ROSCon2023Demo](https://github.com/RobotecAI/ROSCon2023Demo)

---

This work is licensed under [Apache License, Version 2.0][apache]. You may elect at your option to use the [MIT License][mit] instead. Contributions must be made under both licenses.

[apache]: https://opensource.org/licenses/Apache-2.0
[mit]: https://opensource.org/licenses/MIT
[apache_shield]: https://img.shields.io/badge/License-Apache_2.0-blue.svg