/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/queue.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/utils.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <HumanWorker/NpcNavigatorBus.h>
#include <HumanWorker/WaypointComponent.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <RecastNavigation/RecastNavigationMeshBus.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>

namespace ROS2::HumanWorker
{
    class NpcNavigatorComponent
        : public AZ::Component
        , private AZ::TickBus::Handler
        , private AzFramework::EntityDebugDisplayEventBus::Handler
        , private RecastNavigation::RecastNavigationMeshNotificationBus::Handler
    {
    public:
        static constexpr  AZ::Crc32 SafetyBubbleTag = AZ_CRC_CE("SafetyBubble");

        AZ_RTTI(NpcNavigatorComponent, "{BA4E4F96-A88C-406B-853B-E05911D190C4}", AZ::Component);

        NpcNavigatorComponent();
        ~NpcNavigatorComponent() override = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;

    protected:
        using PublisherPtr = std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>>;

        // Internal state of the navigation
        enum class NavigationState
        {
            Idle,
            Navigate,
            Rotate,
        };

        struct Speed
        {
            float m_linear{ 0.0f }, m_angular{ 0.0f };
        };

        struct GoalPose
        {
            AZ::Vector3 m_position{};
            AZ::Vector3 m_direction{};
        };

        // Helper functions for navigation and component setup.
        // Assumes that the argument vectors lie in the XY plane.
        static float GetSignedAngleBetweenUnitVectors(const AZ::Vector3& unitVector1, const AZ::Vector3& unitVector2);
        static AZ::Transform GetEntityTransform(const AZ::EntityId& entityId);
        static NpcNavigatorComponent::PublisherPtr CreatePublisher(
            ROS2FrameComponent* frame, const ROS2::TopicConfiguration& topicConfiguration);
        static bool IsClose(const AZ::Vector3& vector1, const AZ::Vector3& vector2, float acceptableDistanceError);
        [[nodiscard]] AZ::Transform GetCurrentTransform() const;

        // Navigation mesh
        static AZ::EntityId GetNavigationMeshEntityId(AZ::EntityId detourNavigationEntity);
        // RecastNavigationMeshNotificationBus overrides
        void OnNavigationMeshUpdated(AZ::EntityId navigationMeshEntity) override;
        void OnNavigationMeshBeganRecalculating(AZ::EntityId navigationMeshEntity) override
        {
        }

        // Path calculations and navigation
        Speed CalculateSpeedForGoal(
            const AZ::Transform& currentTransform,
            const GoalPose& goal,
            const AZ::Vector3& startPosition,
            const Speed& maxSpeed,
            float crossTrackFactor);
        [[nodiscard]] AZStd::vector<GoalPose> ConstructGoalPath(
            const AZStd::vector<AZ::Vector3>& positionPath, const AZ::Quaternion& waypointOrientation) const;
        AZStd::vector<AZ::Vector3> FindPathBetweenPositions(const AZ::Vector3& currentPosition, const AZ::Vector3& goalPosition);

        // Virtual functions to be implemented by derived classes with different waypoint definitions
        //! Try to find a path to the next goal.
        //! If no path can be found or the goal is unreachable, return an empty vector.
        //! Otherwise, return a vector of goal poses that the NPC should follow.
        virtual AZStd::vector<GoalPose> TryFindGoalPath() = 0;

        //! Calculate the speed of the NPC based on the current state.
        //! The speed is calculated based on the current goal and the current position of the NPC.
        //! @param deltaTime The time since the last calculation.
        virtual NpcNavigatorComponent::Speed CalculateSpeed(float deltaTime) = 0;

        //! Recalculate the current goal path.
        //! This function should be called when a new path needs to be calculated.
        //! It should reset the current state and create a new goal path.
        virtual void RecalculateCurrentGoalPath() = 0;

        // ROS2 communication
        void Publish(Speed speed);

        // AZ::TickBus overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // EntityDebugDisplayEventBus overrides
        void DisplayEntityViewport(const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

        bool UseExplicitlyDefinedNavigationMesh() const;

        NavigationState m_state{ NavigationState::Navigate };
        AZ::EntityId m_navigationEntity;

        // Navigation path variables
        AZStd::vector<GoalPose> m_goalPath;
        size_t m_goalIndex{ 0LU };
        AZ::Vector3 m_startPosition{};

        // Configuration variables
        float m_linearSpeed{ 1.5f };
        float m_angularSpeed{ 1.0f };
        float m_crossTrackFactor{ 0.1f };
        float m_acceptableDistanceError{ 0.5f };
        float m_acceptableAngleError{ 0.1f };
        bool m_debugMode{ false };

        bool m_useTagsForNavigationMesh{ false };

        bool m_preventWalkingInCircle{ false };

        // ROS2 communication variables
        TopicConfiguration m_twistTopicConfiguration;
        PublisherPtr m_publisher;
    };
} // namespace ROS2::HumanWorker
