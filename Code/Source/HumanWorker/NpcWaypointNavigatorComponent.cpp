/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <HumanWorker/NpcWaypointNavigatorComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <HumanWorker/WaypointBus.h>

namespace HumanWorker
{
    void NpcWaypointNavigatorComponent::Reflect(AZ::ReflectContext* context)
    {
        NpcNavigatorComponent::Reflect(context);

        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<NpcWaypointNavigatorComponent, NpcNavigatorComponent>()
                ->Version(0)
                ->Field("Restart on traversed", &NpcWaypointNavigatorComponent::m_restartOnTraversed)
                ->Field("Waypoints", &NpcWaypointNavigatorComponent::m_waypointEntities);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext
                    ->Class<NpcWaypointNavigatorComponent>(
                        "Npc Waypoint Navigator",
                        "Component used for navigating an npc along a selected waypoint path."
                        "It processes paths and publishes twist messages")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "HumanWorker")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &NpcWaypointNavigatorComponent::m_restartOnTraversed, "Restart on traversed", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NpcWaypointNavigatorComponent::m_waypointEntities, "Waypoints", "");
            }
        }
        if (auto* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<NpcNavigatorRequestBus>("NpcNavigatorRequestBus")
                ->Attribute(AZ::Script::Attributes::Category, "HumanWorker")
                ->Event("SelectWaypointPath", &NpcNavigatorRequestBus::Events::SelectWaypointPath)
                ->Event("GoToLocation", &NpcNavigatorRequestBus::Events::GoToLocation);
        }
    }

    void NpcWaypointNavigatorComponent::Activate()
    {
        NpcNavigatorComponent::Activate();
        NpcNavigatorRequestBus::Handler::BusConnect(GetEntityId());

        m_startPosition = GetCurrentTransform().GetTranslation();
    }

    void NpcWaypointNavigatorComponent::Deactivate()
    {
        NpcNavigatorRequestBus::Handler::BusDisconnect();
        NpcNavigatorComponent::Deactivate();
    }

    AZStd::vector<NpcNavigatorComponent::GoalPose> NpcWaypointNavigatorComponent::TryFindGoalPath()
    {
        if (m_waypointIndex >= m_waypointEntities.size())
        {
            return {};
        }

        const AZ::EntityId WaypointEntity = m_waypointEntities[m_waypointIndex];
        const AZStd::vector<AZ::Vector3> PositionPath =
            FindPathBetweenPositions(GetCurrentTransform().GetTranslation(), GetEntityTransform(WaypointEntity).GetTranslation());
        if (PositionPath.empty())
        {
            return {};
        }

        m_waypointConfiguration = FetchWaypointConfiguration(WaypointEntity);
        return ConstructGoalPath(PositionPath, GetEntityTransform(WaypointEntity).GetRotation());
    }

    NpcWaypointNavigatorComponent::Speed NpcWaypointNavigatorComponent::CalculateSpeed(float deltaTime)
    {
        switch (m_state)
        {
        case NavigationState::Idle:
            if ((m_waypointConfiguration.m_idleTime -= deltaTime) <= 0.0f)
            {
                m_goalIndex = 0;
                m_waypointIndex++;

                if (m_waypointIndex >= m_waypointEntities.size() && m_restartOnTraversed)
                {
                    m_waypointIndex = 0;
                }

                m_goalPath.clear();
                m_waypointConfiguration = FetchWaypointConfiguration(m_waypointEntities[m_waypointIndex]);
                m_state = NavigationState::Navigate;

                m_startPosition = GetCurrentTransform().GetTranslation();
            }
            return {};
        case NavigationState::Rotate:
            {
                AZ_Assert(
                    m_goalIndex == m_goalPath.size(), "The Npc Navigator component is in an invalid state due to programmer's error.");
                const float BearingError = GetSignedAngleBetweenUnitVectors(
                    GetCurrentTransform().GetRotation().TransformVector(AZ::Vector3::CreateAxisX()),
                    m_goalPath[m_goalIndex - 1].m_direction);

                if (AZStd::abs(BearingError) < m_acceptableAngleError)
                {
                    m_state = NavigationState::Idle;
                    NpcNavigatorNotificationBus::Event(
                        GetEntityId(), &HumanWorker::NpcNavigatorNotifications::OnWaypointReached, m_waypointConfiguration);
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
                m_goalIndex++;
                if (m_goalIndex == m_goalPath.size())
                {
                    if (m_waypointConfiguration.m_orientationCaptured)
                    {
                        m_state = NavigationState::Rotate;
                    }
                    else
                    {
                        m_state = NavigationState::Idle;
                        NpcNavigatorNotificationBus::Event(
                            GetEntityId(), &HumanWorker::NpcNavigatorNotifications::OnWaypointReached, m_waypointConfiguration);
                    }
                    return {};
                }
                m_startPosition = m_goalPath[m_goalIndex - 1].m_position;
            }

            return CalculateSpeedForGoal(
                GetCurrentTransform(),
                m_goalPath[m_goalIndex],
                m_startPosition,
                { .m_linear = m_linearSpeed, .m_angular = m_angularSpeed },
                m_crossTrackFactor);
        }
    }

    void NpcWaypointNavigatorComponent::RecalculateCurrentGoalPath()
    {
        if (m_waypointIndex == m_waypointEntities.size())
        {
            return;
        }

        m_goalIndex = 0;
        m_goalPath = TryFindGoalPath();
    }

    WaypointConfiguration NpcWaypointNavigatorComponent::FetchWaypointConfiguration(AZ::EntityId waypointEntityId)
    {
        WaypointConfiguration waypointConfiguration;
        WaypointRequestBus::EventResult(waypointConfiguration, waypointEntityId, &WaypointRequests::GetConfiguration);
        return waypointConfiguration;
    }

    void NpcWaypointNavigatorComponent::SelectWaypointPath(const AZStd::vector<AZ::EntityId>& waypointEntityIds)
    {
        m_goalIndex = 0;
        m_goalPath.clear();
        m_state = NavigationState::Navigate;
        m_waypointIndex = 0;
        m_waypointEntities.clear();
        m_waypointEntities = waypointEntityIds;
    }

    void NpcWaypointNavigatorComponent::GoToLocation(const AZ::EntityId& location)
    {
        if (m_waypointEntities.size() == 1 && m_waypointEntities[0] == location)
        {
            return;
        }
        m_goalIndex = 0;
        m_goalPath.clear();
        m_state = NavigationState::Navigate;
        m_waypointIndex = 0;
        m_waypointEntities.clear();
        m_waypointEntities.push_back(location);
        m_restartOnTraversed = false;
    }
} // namespace HumanWorker
