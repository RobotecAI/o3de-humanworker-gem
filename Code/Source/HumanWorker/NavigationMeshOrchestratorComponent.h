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
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <ROS2/Communication/TopicConfiguration.h>

namespace HumanWorker
{
    class NavigationMeshOrchestratorComponent
        : public AZ::Component
        , private AZ::TickBus::Handler
        , private AZ::EntityBus::Handler
    {
    public:
        AZ_COMPONENT(NavigationMeshOrchestratorComponent, "{fb2eabfd-cafb-4fe2-999d-9faec09fb9ba}", AZ::Component);

        NavigationMeshOrchestratorComponent() = default;
        ~NavigationMeshOrchestratorComponent() override = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
        {
            required.push_back(AZ_CRC_CE("RecastNavigationMeshComponent"));
        }

        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;

    private:
        // AZ::TickBus overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // AZ::EntityBus overrides
        void OnEntityActivated(const AZ::EntityId&) override;

        void UpdateNavigationMesh();

        bool m_shouldUpdate{ false };
        float m_updateFrequency{ 0.1f }; //!< In Hertz.
        float m_elapsedTime{ 0.0f }; //!< In seconds.
        bool m_initialUpdate{ true };
        int m_delayedTickUpdate{ 0 };
        bool m_delayedTickUpdateActive{ false };
    };
} // namespace HumanWorker
