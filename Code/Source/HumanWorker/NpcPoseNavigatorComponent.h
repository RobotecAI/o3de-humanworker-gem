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
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Time/ITime.h>
#include <AzCore/std/containers/queue.h>
#include <AzCore/std/parallel/mutex.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ROS2::HumanWorker
{
    class NpcPoseNavigatorComponent : public NpcNavigatorComponent
    {
    public:
        AZ_COMPONENT(NpcPoseNavigatorComponent, "{9D90BDCB-5353-485C-93A4-B1FF63862380}", NpcNavigatorComponent);

        NpcPoseNavigatorComponent();
        ~NpcPoseNavigatorComponent() override = default;

        static void Reflect(AZ::ReflectContext* context);

        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;

    private:
        // NpcNavigatorComponent overrides
        AZStd::vector<GoalPose> TryFindGoalPath() override;
        NpcPoseNavigatorComponent::Speed CalculateSpeed(float deltaTime) override;
        void RecalculateCurrentGoalPath() override;

        AZStd::queue<GoalPose> m_goalPoseQueue;
        AZStd::mutex m_goalPoseQueueMutex;

        bool m_rotateToPoseHeading = true;
        ROS2::TopicConfiguration m_poseTopicConfiguration;
        std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> m_poseSubscription = nullptr;

        std::unique_ptr<tf2_ros::Buffer> m_tfBuffer = nullptr;
        std::unique_ptr<tf2_ros::TransformListener> m_tfListener = nullptr;
        AZStd::string m_worldFrame;
    };
} // namespace ROS2::HumanWorker
