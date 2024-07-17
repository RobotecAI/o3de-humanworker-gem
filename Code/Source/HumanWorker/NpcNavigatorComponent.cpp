/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <HumanWorker/NpcNavigatorComponent.h>

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/EBus/Results.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <HumanWorker/WaypointBus.h>
#include <LmbrCentral/Scripting/TagComponentBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <RecastNavigation/DetourNavigationBus.h>
#include <RecastNavigation/RecastNavigationMeshBus.h>

namespace ROS2::HumanWorker
{
    NpcNavigatorComponent::NpcNavigatorComponent()
    {
        m_twistTopicConfiguration.m_topic = "cmd_vel";
        m_twistTopicConfiguration.m_type = "geometry_msgs::msg::Twist";
    }

    void NpcNavigatorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<NpcNavigatorComponent, AZ::Component>()
                ->Version(1)
                ->Field("Debug Mode", &NpcNavigatorComponent::m_debugMode)
                ->Field("Detour Navigation Entity", &NpcNavigatorComponent::m_navigationEntity)
                ->Field("Twist Topic Configuration", &NpcNavigatorComponent::m_twistTopicConfiguration)
                ->Field("Linear Speed", &NpcNavigatorComponent::m_linearSpeed)
                ->Field("Angular Speed", &NpcNavigatorComponent::m_angularSpeed)
                ->Field("Cross Track Factor", &NpcNavigatorComponent::m_crossTrackFactor)
                ->Field("Acceptable Distance Error", &NpcNavigatorComponent::m_acceptableDistanceError)
                ->Field("Acceptable Angle Error", &NpcNavigatorComponent::m_acceptableAngleError)
                ->Field("UseTag", &NpcNavigatorComponent::m_useTagsForNavigationMesh);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext
                    ->Class<NpcNavigatorComponent>(
                        "Npc Navigator base",
                        "Component used for navigating an npc along a path."
                        "It processes paths and publishes twist messages")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NpcNavigatorComponent::m_debugMode, "Debug Mode", "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &NpcNavigatorComponent::m_useTagsForNavigationMesh,
                        "Use tag for navigation mesh",
                        "If true, the navigation mesh entity is found by tag, otherwise it is explicitly defined by Entity Id.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &NpcNavigatorComponent::m_navigationEntity,
                        "Detour Navigation Entity",
                        "Entity with the Detour Navigation Component")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &NpcNavigatorComponent::UseExplicitlyDefinedNavigationMesh)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &NpcNavigatorComponent::m_twistTopicConfiguration, "Topic Configuration", "")
                    ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "Twist control topic")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NpcNavigatorComponent::m_linearSpeed, "Linear Speed", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NpcNavigatorComponent::m_angularSpeed, "Angular Speed", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NpcNavigatorComponent::m_crossTrackFactor, "Cross Track Factor", "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &NpcNavigatorComponent::m_acceptableDistanceError, "Acceptable Distance Error", "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &NpcNavigatorComponent::m_acceptableAngleError, "Acceptable Angle Error", "");
            }
        }
    }

    void NpcNavigatorComponent::Activate()
    {
        // Ensure that all required components are activated
        AZ::SystemTickBus::QueueFunction(
            [this]()
            {
                if (m_useTagsForNavigationMesh)
                {
                    const AZ::EntityId currentId = GetEntityId();

                    // Find all entities that have the Detour Navigation Component and are tagged with the same tag as this entity
                    LmbrCentral::Tags tags;
                    LmbrCentral::TagComponentRequestBus::EventResult(
                        tags, GetEntityId(), &LmbrCentral::TagComponentRequestBus::Events::GetTags);
                    for (const auto& tag : tags)
                    {
                        bool foundEntity = false;
                        AZ::EBusAggregateResults<AZ::EntityId> tagSearchResults;
                        LmbrCentral::TagGlobalRequestBus::EventResult(
                            tagSearchResults, tag, &LmbrCentral::TagGlobalRequestBus::Events::RequestTaggedEntities);

                        if (!tagSearchResults.values.empty())
                        {
                            for (const auto& entityId : tagSearchResults.values)
                            {
                                // Only a valid entity with a DetourNavigationRequestBus handler can be used
                                if (entityId.IsValid() && entityId != currentId &&
                                    RecastNavigation::DetourNavigationRequestBus::HasHandlers(entityId))
                                {
                                    m_navigationEntity = entityId;
                                    foundEntity = true;
                                    break;
                                }
                            }
                        }
                        if (foundEntity)
                        {
                            break;
                        }
                    }
                }

                RecastNavigation::RecastNavigationMeshNotificationBus::Handler::BusConnect(GetNavigationMeshEntityId(m_navigationEntity));
                if (m_debugMode)
                {
                    AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(m_entity->GetId());
                }
            });

        m_publisher =
            CreatePublisher(ROS2::Utils::GetGameOrEditorComponent<ROS2::ROS2FrameComponent>(GetEntity()), m_twistTopicConfiguration);

        AZ::TickBus::Handler::BusConnect();

        m_startPosition = GetCurrentTransform().GetTranslation();
    }

    void NpcNavigatorComponent::Deactivate()
    {
        if (m_debugMode)
        {
            AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect(m_entity->GetId());
        }
        RecastNavigation::RecastNavigationMeshNotificationBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
    }

    bool NpcNavigatorComponent::IsClose(AZ::Vector3 vector1, AZ::Vector3 vector2, float acceptableDistanceError)
    {
        AZ::Vector2 vector21{ vector1.GetX(), vector1.GetY() };
        AZ::Vector2 vector22{ vector2.GetX(), vector2.GetY() };
        return vector21.GetDistance(vector22) < acceptableDistanceError;
    }

    AZ::Transform NpcNavigatorComponent::GetEntityTransform(AZ::EntityId entityId)
    {
        AZ::Transform currentTransform{ AZ::Transform::CreateIdentity() };
        AZ::TransformBus::EventResult(currentTransform, entityId, &AZ::TransformBus::Events::GetWorldTM);
        return currentTransform;
    }

    float NpcNavigatorComponent::GetSignedAngleBetweenUnitVectors(AZ::Vector3 unitVector1, AZ::Vector3 unitVector2)
    {
        return AZ::Atan2(unitVector1.Cross(unitVector2).Dot(AZ::Vector3::CreateAxisZ()), unitVector1.Dot(unitVector2));
    }

    AZ::EntityId NpcNavigatorComponent::GetNavigationMeshEntityId(AZ::EntityId detourNavigationEntity)
    {
        AZ::EntityId navigationMeshEntityId;
        RecastNavigation::DetourNavigationRequestBus::EventResult(
            navigationMeshEntityId, detourNavigationEntity, &RecastNavigation::DetourNavigationRequests::GetNavigationMeshEntity);
        return navigationMeshEntityId;
    }

    NpcNavigatorComponent::Speed NpcNavigatorComponent::CalculateSpeedForGoal(
        const AZ::Transform& currentTransform, GoalPose goal, AZ::Vector3 startPosition, Speed maxSpeed, float crossTrackFactor)
    {
        const AZ::Vector3 RobotPosition = currentTransform.GetTranslation();
        const AZ::Vector3 RobotDirection = currentTransform.GetBasisX().GetNormalized();

        float bearingError = 0.0f;
        if (goal.m_position != RobotPosition)
        {
            const AZ::Vector3 GoalDirection = (goal.m_position - RobotPosition).GetNormalized();
            bearingError = GetSignedAngleBetweenUnitVectors(RobotDirection, GoalDirection);
        }

        const AZ::Vector3 zeroZ = AZ::Vector3::CreateOne() - AZ::Vector3::CreateAxisZ(1);

        const AZ::Vector3 routeVector = (goal.m_position - startPosition) * zeroZ;
        const AZ::Vector3 positionVector = (RobotPosition - startPosition) * zeroZ;
        const AZ::Vector3 crossProduct = routeVector.Cross(positionVector);
        const float crossTrackError = (crossProduct.GetLength() / routeVector.GetLength()) * (crossProduct.GetZ() > 0 ? 1 : -1);

        return Speed{ .m_linear = maxSpeed.m_linear, .m_angular = bearingError * maxSpeed.m_angular - crossTrackError * crossTrackFactor };
    }

    NpcNavigatorComponent::PublisherPtr NpcNavigatorComponent::CreatePublisher(
        ROS2FrameComponent* frame, const ROS2::TopicConfiguration& topicConfiguration)
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        const auto& topicName = ROS2::ROS2Names::GetNamespacedName(frame->GetNamespace(), topicConfiguration.m_topic);
        const auto& qos = topicConfiguration.GetQoS();
        return ros2Node->create_publisher<geometry_msgs::msg::Twist>(topicName.data(), qos);
    }

    void NpcNavigatorComponent::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        if (m_goalPath.empty() && (m_goalPath = TryFindGoalPath()).empty())
        {
            return;
        }

        Publish(CalculateSpeed(deltaTime));
    }

    void NpcNavigatorComponent::DisplayEntityViewport(
        const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        for (size_t pointIndex = 0; pointIndex < m_goalPath.size(); ++pointIndex)
        {
            if (pointIndex == m_goalIndex)
            {
                debugDisplay.SetColor(AZ::Colors::YellowGreen);
                debugDisplay.DrawBall(m_goalPath[pointIndex].m_position, 0.1f);
            }
            else if (pointIndex == (m_goalPath.size() - 1))
            {
                debugDisplay.SetColor(AZ::Colors::Green);
                debugDisplay.DrawBall(m_goalPath[pointIndex].m_position, 0.2f);
            }

            if (pointIndex != 0)
            {
                debugDisplay.SetColor(AZ::Colors::Yellow);
                debugDisplay.DrawArrow(m_goalPath[pointIndex - 1].m_position, m_goalPath[pointIndex].m_position, 0.2f);
            }

            debugDisplay.SetColor(AZ::Colors::Red);
            debugDisplay.DrawArrow(
                m_goalPath[pointIndex].m_position, m_goalPath[pointIndex].m_position + m_goalPath[pointIndex].m_direction, 0.2f);
        }
    }

    void NpcNavigatorComponent::OnNavigationMeshUpdated(AZ::EntityId navigationMeshEntity)
    {
        RecalculateCurrentGoalPath();
    }

    AZ::Transform NpcNavigatorComponent::GetCurrentTransform() const
    {
        return GetEntityTransform(GetEntityId());
    }

    AZStd::vector<NpcNavigatorComponent::GoalPose> NpcNavigatorComponent::ConstructGoalPath(
        const AZStd::vector<AZ::Vector3>& positionPath, const AZ::Quaternion& waypointOrientation) const
    {
        AZStd::vector<GoalPose> goalPath;
        for (size_t i = 0; i < positionPath.size(); ++i)
        {
            AZ::Vector3 direction = AZ::Vector3::CreateZero();

            // get direction
            auto it = positionPath.begin() + i;
            auto nextIt = it + 1;
            if (nextIt != positionPath.end())
            {
                direction = (*nextIt - *it).GetNormalized();
            }
            else
            {
                direction = waypointOrientation.TransformVector(AZ::Vector3::CreateAxisX());
            }
            goalPath.push_back({ .m_position = positionPath[i], .m_direction = direction });
        }
        return goalPath;
    }

    AZStd::vector<AZ::Vector3> NpcNavigatorComponent::FindPathBetweenPositions(AZ::Vector3 currentPosition, AZ::Vector3 goalPosition)
    {
        if (!GetNavigationMeshEntityId(m_navigationEntity).IsValid())
        {
            AZ_Error(__func__, false, "Unable to query the Detour Navigation Request Bus.");
            return {};
        }

        AZStd::vector<AZ::Vector3> path;
        RecastNavigation::DetourNavigationRequestBus::EventResult(
            path, m_navigationEntity, &RecastNavigation::DetourNavigationRequests::FindPathBetweenPositions, currentPosition, goalPosition);

        return path;
    }

    void NpcNavigatorComponent::Publish(Speed speed)
    {
        geometry_msgs::msg::Twist cmdVelMessage;
        cmdVelMessage.linear.x = speed.m_linear;
        cmdVelMessage.angular.z = speed.m_angular;

        m_publisher->publish(cmdVelMessage);
    }

    bool NpcNavigatorComponent::UseExplicitlyDefinedNavigationMesh() const
    {
        return !m_useTagsForNavigationMesh;
    }
} // namespace ROS2::HumanWorker
