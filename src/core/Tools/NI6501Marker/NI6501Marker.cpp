/* $BEGIN_BCI2000_LICENSE$
 *
 * This file is part of BCI2000, a platform for real-time bio-signal research.
 * [ Copyright (C) 2000-2026: BCI2000 team and many external contributors ]
 *
 * BCI2000 is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * BCI2000 is distributed in the hope that it will be useful, but
 *                         WITHOUT ANY WARRANTY
 * - without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * $END_BCI2000_LICENSE$
 */
/******************************************************************************
 * Program: NI6501Marker                                                      *
 * Comment: Standalone console tool.                                          *
 *                                                                            *
 *          Press 'K' -> short HIGH pulse (a marker) on P0.0.                 *
 *          Press 'L' -> short HIGH pulse (a marker) on P0.1.                 *
 *          Press ESC -> quit.                                                *
 *          Each pulse prints one confirmation line to the console.           *
 *                                                                            *
 * Wiring (NI 6501 -> trigger box):                                           *
 *          GND  -> common ground                                             *
 *          P0.0 -> marker line 0  (triggered by K)                           *
 *          P0.1 -> marker line 1  (triggered by L)                           *
 *                                                                            *
 * Implementation:                                                            *
 *   Reuses BCI2000's NIDAQ-MX "imports" wrapper, which loads nicaiu.dll      *
 *   (the NI-DAQmx runtime) dynamically. No NIDAQmx.lib link and no NI        *
 *   include paths are required at build time; only the NI-DAQmx driver       *
 *   needs to be installed at run time.                                       *
 ******************************************************************************/

#include <cstdio>
#include <cstdlib>
#include <conio.h>     // _getch  -- immediate console key read (Windows)
#include <windows.h>   // Sleep

// This header redirects DAQmx function names to dynamically loaded pointers
// and, at its end, includes NIDAQmx.h, so TaskHandle / DAQmx_Val_* etc. are
// available.
#include "NIDAQmx.imports.h"

// Provided by NIDAQmx.imports.cpp: checks whether nicaiu.dll was loaded.
namespace Dylib { bool nicaiu_Loaded(); }

// ===========================================================================
//  CONFIG  -- the only things you may need to change
// ===========================================================================

// (1) Device name. Check NI MAX (Measurement & Automation Explorer) for the
//     name assigned to your 6501. A freshly installed board is usually "Dev1".
static const char* kDevice = "Dev1";

// (2) The two wired output lines, as one digital-output task.
//     "Dev1/port0/line0:1" means P0.0 and P0.1. If you change kDevice above,
//     change the prefix here too.
static const char* kChannel = "Dev1/port0/line0:1";

// (3) High-level pulse width, in milliseconds.
static const int kPulseMs = 200;

// (4) Trigger keys.
static const char kKeyLine0 = 'K';   // K -> pulse on P0.0
static const char kKeyLine1 = 'L';   // L -> pulse on P0.1

// ===========================================================================

// Check a DAQmx return code; print the message and exit on failure.
static void CheckError(int32 error)
{
    if (DAQmxFailed(error))
    {
        char buf[2048] = {0};
        DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
        std::printf("\n[DAQmx ERROR] %s\n", buf);
        std::exit(1);
    }
}

// Write both lines to the given levels. l0 -> P0.0, l1 -> P0.1 (0=low,1=high).
static void WriteLines(TaskHandle task, uInt8 l0, uInt8 l1)
{
    uInt8 data[2] = { l0, l1 };
    // numSampsPerChan=1, autoStart=1, timeout=10s, grouped by channel.
    CheckError(DAQmxWriteDigitalLines(task, 1, 1, 10.0,
                                      DAQmx_Val_GroupByChannel, data, NULL, NULL));
}

// Emit a short pulse on one line: high -> wait -> low. lineIndex: 0=P0.0,1=P0.1.
static void Pulse(TaskHandle task, int lineIndex)
{
    if (lineIndex == 0) WriteLines(task, 1, 0);   // P0.0 high, P0.1 low
    else                WriteLines(task, 0, 1);   // P0.0 low,  P0.1 high
    Sleep(kPulseMs);
    WriteLines(task, 0, 0);                        // both lines low
}

int main()
{
    // Make sure the NI-DAQmx runtime (nicaiu.dll) is loaded.
    if (!Dylib::nicaiu_Loaded())
    {
        std::printf("Could not load the NI-DAQmx runtime (nicaiu.dll). "
                    "Please make sure the NI-DAQmx driver is installed.\n");
        return 1;
    }

    TaskHandle task = 0;

    // Create the digital-output task on P0.0 and P0.1, then start it.
    CheckError(DAQmxCreateTask("", &task));
    CheckError(DAQmxCreateDOChan(task, kChannel, "", DAQmx_Val_ChanForAllLines));
    CheckError(DAQmxStartTask(task));

    // Initialize both lines low.
    WriteLines(task, 0, 0);

    std::printf("NI6501 Marker ready.\n");
    std::printf("Device  : %s\n", kDevice);
    std::printf("Channel : %s\n", kChannel);
    std::printf("Pulse   : %d ms\n\n", kPulseMs);
    std::printf("  '%c' -> marker on P0.0\n", kKeyLine0);
    std::printf("  '%c' -> marker on P0.1\n", kKeyLine1);
    std::printf("  ESC -> quit\n\n");

    int count0 = 0, count1 = 0;
    for (;;)
    {
        int ch = _getch();          // block until a key is pressed
        if (ch == 27)               // ESC
            break;

        // Accept both upper- and lower-case (xor 0x20 swaps case).
        if (ch == kKeyLine0 || ch == (kKeyLine0 ^ 0x20))
        {
            Pulse(task, 0);
            std::printf("[P0.0 marker #%d] pulse sent\n", ++count0);
        }
        else if (ch == kKeyLine1 || ch == (kKeyLine1 ^ 0x20))
        {
            Pulse(task, 1);
            std::printf("[P0.1 marker #%d] pulse sent\n", ++count1);
        }
    }

    // Clean up: drive both lines low, stop and clear the task.
    WriteLines(task, 0, 0);
    DAQmxStopTask(task);
    DAQmxClearTask(task);

    std::printf("\nDone. P0.0=%d markers, P0.1=%d markers.\n", count0, count1);
    return 0;
}