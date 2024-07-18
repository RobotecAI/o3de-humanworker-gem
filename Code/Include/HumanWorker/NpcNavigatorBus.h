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

#include <HumanWorker/WaypointBus.h>

namespace ROS2::HumanWorker
{
    class NpcNavigatorRequests
    {
    public:
        AZ_RTTI(NpcNavigatorRequests, "{31d0a864-9d15-4ad7-a597-a4573937957d}");
        virtual ~NpcNavigatorRequests() = default;

        //! Selects a path for the npc to navigate.
        //! @param waypointEntityIds The entity ids of the waypoints that make up the path.
        virtual void SelectWaypointPath(const AZStd::vector<AZ::EntityId>& waypointEntityIds) = 0;
    };

    class NpcNavigatorRequestBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        using BusIdType = AZ::EntityId;
        //////////////////////////////////////////////////////////////////////////
    };

    using NpcNavigatorRequestBus = AZ::EBus<NpcNavigatorRequests, NpcNavigatorRequestBusTraits>;

    class NpcNavigatorNotifications
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        using BusIdType = AZ::EntityId;
        //////////////////////////////////////////////////////////////////////////

        //! Notification that the npc has reached a waypoint.
        //! @param waypointConfig The configuration of the waypoint that was reached.
        virtual void OnWaypointReached([[maybe_unused]] WaypointConfiguration waypointConfig) {}
    };

    using NpcNavigatorNotificationBus = AZ::EBus<NpcNavigatorNotifications>;
} // namespace ROS2::HumanWorker
