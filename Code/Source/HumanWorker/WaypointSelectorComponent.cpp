/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <HumanWorker/NpcNavigatorBus.h>
#include <HumanWorker/WaypointSelectorComponent.h>

namespace HumanWorker
{
    void WaypointSelectorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<WaypointSelectorComponent, AZ::Component>()
                ->Version(1)
                ->Field("Seed", &WaypointSelectorComponent::m_seed)
                ->Field("Human Npcs", &WaypointSelectorComponent::m_humanNpcs)
                ->Field("Waypoint Entities", &WaypointSelectorComponent::m_waypoints);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                // clang-format off
                editContext
                    ->Class<WaypointSelectorComponent>(
                        "Waypoint Selector",
                        "Component used to construct a randomized path from a provided waypoint set for selected Npc Navigators.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "HumanWorker")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &WaypointSelectorComponent::m_seed, "Seed", "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WaypointSelectorComponent::m_humanNpcs,
                        "HumanWorkers",
                        "Entities with the NpcNavigator components.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WaypointSelectorComponent::m_waypoints,
                        "Waypoints",
                        "Entities with the Waypoint components.");
                // clang-format on
            }
        }
    }

    void WaypointSelectorComponent::Activate()
    {
        m_mersenneTwister.seed(m_seed);
        for (AZ::EntityId humanNpcEntityId : m_humanNpcs)
        {
            AZ::EntityBus::MultiHandler::BusConnect(humanNpcEntityId);
        }
    }

    void WaypointSelectorComponent::Deactivate()
    {
        for (size_t i = m_humanNpcs.size() - 1; i <= 0; --i)
        {
            AZ::EntityBus::MultiHandler::BusDisconnect(m_humanNpcs[i]);
        }
    }

    void WaypointSelectorComponent::OnEntityActivated(const AZ::EntityId& entityId)
    {
        const AZStd::vector<AZ::EntityId> waypoints = SelectWaypointPah();
        NpcNavigatorRequestBus::Event(entityId, &NpcNavigatorRequests::SelectWaypointPath, waypoints);
    }

    AZStd::vector<AZ::EntityId> WaypointSelectorComponent::SelectWaypointPah()
    {
        if (m_waypoints.empty())
        {
            AZ_Printf(__func__, "No waypoint entities to select from.") return {};
        }

        AZStd::vector<AZ::EntityId> waypointPath = m_waypoints;
        std::shuffle(waypointPath.begin(), waypointPath.end(), m_mersenneTwister);

        return waypointPath;
    }
} // namespace HumanWorker