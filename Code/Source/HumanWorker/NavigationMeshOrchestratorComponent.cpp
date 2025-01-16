/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <HumanWorker/NavigationMeshOrchestratorComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <RecastNavigation/RecastNavigationMeshBus.h>

namespace HumanWorker
{
    void NavigationMeshOrchestratorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<NavigationMeshOrchestratorComponent, AZ::Component>()
                ->Version(0)
                ->Field("Asynchronous NavMesh Update", &NavigationMeshOrchestratorComponent::m_shouldUpdate)
                ->Field("NavMesh Update Frequency", &NavigationMeshOrchestratorComponent::m_updateFrequency)
                ->Field("UseDelayedUpdate", &NavigationMeshOrchestratorComponent::m_delayedTickUpdateActive)
                ->Field("DelayedUpdate", &NavigationMeshOrchestratorComponent::m_delayedTickUpdate);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext->Class<NavigationMeshOrchestratorComponent>("Navigation Orchestrator", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "HumanWorker")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &NavigationMeshOrchestratorComponent::m_shouldUpdate,
                        "Should the mesh be updated",
                        "")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &NavigationMeshOrchestratorComponent::m_updateFrequency,
                        "NavMesh Update Frequency",
                        "")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Visibility, &NavigationMeshOrchestratorComponent::m_shouldUpdate)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &NavigationMeshOrchestratorComponent::m_delayedTickUpdateActive,
                        "Use Delayed Update",
                        "")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &NavigationMeshOrchestratorComponent::m_delayedTickUpdate,
                        "Ticks to update after",
                        "")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &NavigationMeshOrchestratorComponent::m_delayedTickUpdateActive);
            }
        }
    }

    void NavigationMeshOrchestratorComponent::Activate()
    {
        AZ::EntityBus::Handler::BusConnect(GetEntityId());
    }

    void NavigationMeshOrchestratorComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        AZ::EntityBus::Handler::BusDisconnect();
    }

    void NavigationMeshOrchestratorComponent::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        if (m_delayedTickUpdateActive)
        {
            if (m_delayedTickUpdate > 0)
            {
                m_delayedTickUpdate--;
            }
            else
            {
                m_delayedTickUpdateActive = false;
                m_initialUpdate = false;
                UpdateNavigationMesh();
            }
        }
        else if (m_initialUpdate)
        {
            // Update the navigation mesh on the first tick if it is not delayed.
            // The navigation mesh requires a manual update and the mesh is not created automatically on the start of the simulation.
            m_initialUpdate = false;
            UpdateNavigationMesh();
        }

        if (m_shouldUpdate && (m_elapsedTime += deltaTime) * m_updateFrequency > 1.0f)
        {
            m_elapsedTime = 0.0f;
            UpdateNavigationMesh();
        }
    }

    void NavigationMeshOrchestratorComponent::OnEntityActivated(const AZ::EntityId&)
    {
        AZ::TickBus::Handler::BusConnect();
    }

    void NavigationMeshOrchestratorComponent::UpdateNavigationMesh()
    {
        bool success = false;
        RecastNavigation::RecastNavigationMeshRequestBus::EventResult(
            success, GetEntityId(), &RecastNavigation::RecastNavigationMeshRequests::UpdateNavigationMeshBlockUntilCompleted);
    }
} // namespace HumanWorker
