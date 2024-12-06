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
#include <AzCore/Serialization/EditContext.h>
#include <HumanWorker/WaypointBus.h>

namespace HumanWorker
{
    //! Component that signifies a waypoint.
    //! The waypoint may be configured to capture its orientation
    //! (only the rotation around the z axis) or require an idle time.
    class WaypointComponent
        : public AZ::Component
        , private WaypointRequestBus::Handler
    {
    public:
        AZ_COMPONENT(WaypointComponent, "{e3acb4eb-bc11-438c-b4da-f9b4b67c7d3b}", AZ::Component);
        WaypointComponent() = default;
        ~WaypointComponent() override = default;

        // clang-format off
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required) {}
        // clang-format on
        static void Reflect(AZ::ReflectContext* context);

        // Component overrides
        void Activate() override;
        void Deactivate() override;

        // WaypointRequestBus overrides
        WaypointConfiguration GetConfiguration() override;

    private:
        WaypointConfiguration m_configuration;
    };
} // namespace HumanWorker
