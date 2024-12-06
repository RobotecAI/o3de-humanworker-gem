/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

namespace HumanWorker
{
    struct WaypointConfiguration
    {
        AZ_TYPE_INFO(WaypointConfiguration, "{7b416d78-90b1-410a-a85a-835a4fc27e4a}");
        static void Reflect(AZ::ReflectContext* context)
        {
            if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
            {
                serializeContext->Class<WaypointConfiguration>()
                    ->Version(0)
                    ->Field("Orientation captured", &WaypointConfiguration::m_orientationCaptured)
                    ->Field("Idle time", &WaypointConfiguration::m_idleTime);

                if (AZ::EditContext* editContext = serializeContext->GetEditContext())
                {
                    editContext->Class<WaypointConfiguration>("Waypoint Configuration", "Waypoint Configuration")
                        ->DataElement(
                            AZ::Edit::UIHandlers::Default,
                            &WaypointConfiguration::m_orientationCaptured,
                            "Orientation captured",
                            "Should the waypoint orientation be captured?")
                        ->DataElement(
                            AZ::Edit::UIHandlers::Default, &WaypointConfiguration::m_idleTime, "Idle time", "Time spent at waypoint.");
                }
            }
        }

        bool m_orientationCaptured{ false };
        float m_idleTime{ 0 };
    };
} // namespace HumanWorker
