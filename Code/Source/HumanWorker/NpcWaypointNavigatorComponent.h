/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <HumanWorker/NpcNavigatorComponent.h>

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/queue.h>
#include <AzCore/std/containers/vector.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>

namespace ROS2::HumanWorker
{
    class NpcWaypointNavigatorComponent
        : public NpcNavigatorComponent
        , private NpcNavigatorRequestBus::Handler
    {
    public:
        AZ_COMPONENT(NpcWaypointNavigatorComponent, "{2B71BDA6-B986-4627-8E68-15821565F503}", AZ::Component);

        NpcWaypointNavigatorComponent() = default;
        ~NpcWaypointNavigatorComponent() override = default;

        static void Reflect(AZ::ReflectContext* context);

        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;

    private:
        // NpcNavigatorComponent overrides
        AZStd::vector<NpcNavigatorComponent::GoalPose> TryFindGoalPath() override;
        NpcNavigatorComponent::Speed CalculateSpeed(float deltaTime) override;
        void RecalculateCurrentGoalPath() override;

        // NpcNavigatorRequestBus overrides
        void SelectWaypointPath(const AZStd::vector<AZ::EntityId>& waypointEntityIds) override;
        void GoToLocation(const AZ::EntityId& location) override;

        static WaypointConfiguration FetchWaypointConfiguration(AZ::EntityId waypointEntityId);

        AZStd::vector<AZ::EntityId> m_waypointEntities;
        size_t m_waypointIndex{ 0LU };
        WaypointConfiguration m_waypointConfiguration;

        bool m_restartOnTraversed{ true };
    };
} // namespace ROS2::HumanWorker
