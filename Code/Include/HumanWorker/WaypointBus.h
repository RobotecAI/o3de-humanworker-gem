/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Serialization/EditContext.h>

namespace ROS2::HumanWorker
{
    struct WaypointConfiguration
    {
        AZ_TYPE_INFO(WaypointConfiguration, "{7b416d78-90b1-410a-a85a-835a4fc27e4a}");
        static void Reflect(AZ::ReflectContext* context)
        {
            if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
            {
                serializeContext->Class<WaypointConfiguration>()
                    ->Version(1)
                    ->Field("Orientation captured", &WaypointConfiguration::m_orientationCaptured)
                    ->Field("Idle time", &WaypointConfiguration::m_idleTime);

                if (AZ::EditContext* editContext = serializeContext->GetEditContext())
                {
                    // clang-format off
                editContext->Class<WaypointConfiguration>("Waypoint Configuration", "Waypoint Configuration")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WaypointConfiguration::m_orientationCaptured,
                        "Orientation captured",
                        "Should the waypoint orientation be captured?")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WaypointConfiguration::m_idleTime,
                        "Idle time",
                        "Time spent at waypoint.");
                    // clang-format on
                }
            }
        }

        bool m_orientationCaptured{ false };
        float m_idleTime{ 0 };
    };


    class WaypointRequests
    {
    public:
        AZ_RTTI(WaypointRequests, "{0c56b5b6-2daf-4b8b-a1a3-7b43a04d3549}");
        virtual ~WaypointRequests() = default;

        virtual WaypointConfiguration GetConfiguration() = 0;
    };

    class WaypointRequestBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        using BusIdType = AZ::EntityId;
        //////////////////////////////////////////////////////////////////////////
    };

    using WaypointRequestBus = AZ::EBus<WaypointRequests, WaypointRequestBusTraits>;
} // namespace ROS2::HumanWorker
