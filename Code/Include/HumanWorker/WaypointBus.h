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
#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <HumanWorker/WaypointConfiguration.h>

namespace HumanWorker
{
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
} // namespace HumanWorker
