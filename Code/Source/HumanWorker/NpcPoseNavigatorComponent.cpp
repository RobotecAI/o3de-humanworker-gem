/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <HumanWorker/NpcPoseNavigatorComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/parallel/lock.h>
#include <AzCore/std/parallel/mutex.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace HumanWorker
{
    NpcPoseNavigatorComponent::NpcPoseNavigatorComponent()
        : NpcNavigatorComponent()
    {
        m_poseTopicConfiguration.m_topic = "goal_pose";
        m_poseTopicConfiguration.m_type = "geometry_msgs::msg::PoseStamped";
    }

    void NpcPoseNavigatorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<NpcPoseNavigatorComponent, NpcNavigatorComponent>()
                ->Version(0)
                ->Field("Pose Topic Configuration", &NpcPoseNavigatorComponent::m_poseTopicConfiguration)
                ->Field("Rotate To Pose Heading", &NpcPoseNavigatorComponent::m_rotateToPoseHeading);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext
                    ->Class<NpcPoseNavigatorComponent>(
                        "Npc Pose Navigator",
                        "Component used for navigating an npc along poses sent with ROS2."
                        "It processes paths and publishes twist messages")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "HumanWorker")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &NpcPoseNavigatorComponent::m_poseTopicConfiguration, "Topic Configuration", "")
                    ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "Pose waypoints topic")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &NpcPoseNavigatorComponent::m_rotateToPoseHeading,
                        "Rotate To Pose Heading",
                        "Rotate the npc to the pose heading on the final pose.");
            }
        }
    }

    void NpcPoseNavigatorComponent::Activate()
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        if (!ros2Node)
        {
            AZ_Error("NpcPoseNavigatorComponent", false, "Failed to get ROS2 node.");
            return;
        }

        const auto* ros2FrameComponent = m_entity->FindComponent<ROS2::ROS2FrameComponent>();
        auto namespacedTopicName = ROS2::ROS2Names::GetNamespacedName(ros2FrameComponent->GetNamespace(), m_poseTopicConfiguration.m_topic);

        const AZStd::string odomFrame = ros2FrameComponent->GetGlobalFrameName();

        m_tfBuffer = std::make_unique<tf2_ros::Buffer>(ros2Node->get_clock());
        m_tfListener = std::make_unique<tf2_ros::TransformListener>(*m_tfBuffer);

        m_poseSubscription = ros2Node->create_subscription<geometry_msgs::msg::PoseStamped>(
            namespacedTopicName.data(),
            m_poseTopicConfiguration.GetQoS(),
            [this, odomFrame](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                AZStd::lock_guard<AZStd::mutex> lock(m_goalPoseQueueMutex);
                geometry_msgs::msg::PoseStamped::SharedPtr goalPoseMsg;

                geometry_msgs::msg::TransformStamped transformStamped;
                if (msg->header.frame_id != "")
                {
                    if (m_tfBuffer->canTransform(odomFrame.c_str(), msg->header.frame_id, tf2::TimePointZero))
                    {
                        transformStamped = m_tfBuffer->lookupTransform(odomFrame.c_str(), msg->header.frame_id, tf2::TimePointZero);
                        AZ::Quaternion rotation = ROS2::ROS2Conversions::FromROS2Quaternion(transformStamped.transform.rotation);
                        AZ::Vector3 translation = ROS2::ROS2Conversions::FromROS2Vector3(transformStamped.transform.translation);

                        translation += ROS2::ROS2Conversions::FromROS2Point(msg->pose.position);
                        rotation *= ROS2::ROS2Conversions::FromROS2Quaternion(msg->pose.orientation);

                        m_goalPoseQueue.push({ translation, rotation.GetEulerRadians() });
                    }
                    else
                    {
                        return;
                    }
                }
                else
                {
                    m_goalPoseQueue.push({ ROS2::ROS2Conversions::FromROS2Point(msg->pose.position),
                                           ROS2::ROS2Conversions::FromROS2Quaternion(msg->pose.orientation).GetEulerRadians() });
                }
            });

        NpcNavigatorComponent::Activate();
    }

    void NpcPoseNavigatorComponent::Deactivate()
    {
        NpcNavigatorComponent::Deactivate();

        m_poseSubscription.reset();
        m_tfBuffer.reset();
        m_tfListener.reset();
    }

    AZStd::vector<NpcNavigatorComponent::GoalPose> NpcPoseNavigatorComponent::TryFindGoalPath()
    {
        if (m_goalPoseQueue.empty())
        {
            return {};
        }

        GoalPose goalPose;
        {
            AZStd::lock_guard<AZStd::mutex> lock(m_goalPoseQueueMutex);
            goalPose = m_goalPoseQueue.front();
        }
        const AZStd::vector<AZ::Vector3> positionPath =
            FindPathBetweenPositions(GetCurrentTransform().GetTranslation(), goalPose.m_position);

        m_startPosition = GetCurrentTransform().GetTranslation();

        if (positionPath.empty())
        {
            AZStd::lock_guard<AZStd::mutex> lock(m_goalPoseQueueMutex);
            m_goalPoseQueue.pop();

            return {};
        }

        return ConstructGoalPath(positionPath, AZ::Quaternion::CreateFromEulerAnglesRadians(goalPose.m_direction));
    }

    NpcPoseNavigatorComponent::Speed NpcPoseNavigatorComponent::CalculateSpeed(float deltaTime)
    {
        switch (m_state)
        {
        case NavigationState::Idle:
            {
                AZStd::lock_guard<AZStd::mutex> lock(m_goalPoseQueueMutex);
                m_goalPoseQueue.pop();
            }
            m_goalPath.clear();
            m_goalIndex = 0;

            m_state = NavigationState::Navigate;

            m_startPosition = GetCurrentTransform().GetTranslation();

            return {};
        case NavigationState::Rotate:
            {
                AZ_Assert(
                    m_goalIndex == m_goalPath.size(), "The Npc Navigator component is in an invalid state due to programmer's error.");

                if (!m_rotateToPoseHeading)
                {
                    m_state = NavigationState::Idle;
                    return {};
                }

                const float BearingError = GetSignedAngleBetweenUnitVectors(
                    GetCurrentTransform().GetRotation().TransformVector(AZ::Vector3::CreateAxisX()),
                    m_goalPath[m_goalIndex - 1].m_direction);

                if (AZStd::abs(BearingError) < m_acceptableAngleError)
                {
                    m_state = NavigationState::Idle;
                    return {};
                }
                else
                {
                    return {
                        .m_linear = 0.0f,
                        .m_angular = m_angularSpeed * BearingError,
                    };
                }
            }
        case NavigationState::Navigate:
            if (IsClose(m_goalPath[m_goalIndex].m_position, GetCurrentTransform().GetTranslation(), m_acceptableDistanceError))
            {
                if ((m_goalIndex + 1) == m_goalPath.size())
                {
                    m_state = NavigationState::Rotate;
                    return {};
                }
                m_startPosition = m_goalPath[m_goalIndex].m_position;

                m_goalIndex++;
            }

            return CalculateSpeedForGoal(
                GetCurrentTransform(),
                m_goalPath[m_goalIndex],
                m_startPosition,
                { .m_linear = m_linearSpeed, .m_angular = m_angularSpeed },
                m_crossTrackFactor);
        }
    }

    void NpcPoseNavigatorComponent::RecalculateCurrentGoalPath()
    {
        m_goalIndex = 0;
        m_goalPath = TryFindGoalPath();
    }
} // namespace HumanWorker
