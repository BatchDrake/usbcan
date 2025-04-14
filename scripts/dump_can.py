#!/usr/bin/env python3
# 
# Copyright (c) 2025 Gonzalo José Carracedo Carballal <BatchDrake@gmail.com>
# 
# Redistribution and use in source and binary forms, with or withoutmodification,
# are permitted provided that the following conditions are met:
# 
#   1. Redistributions of source code must retain the above copyright notice,
#      this list of conditions and the following disclaimer.
# 
#   2. Redistributions in binary form must reproduce the above copyright notice,
#      this list of conditions and the following disclaimer in the documentation
#      and/or other materials provided with the distribution.
# 
#   3. Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from this
#      software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

import can
from datetime import datetime

with can.Bus(interface='socketcan', channel='vcan0') as bus:
  for msg in bus:
    ts   = datetime.now().timestamp() # What time is it (now)
    id   = msg.arbitration_id         # CAN ID
    data = msg.data.hex()             # Message data (up to 8 bytes)
    print(f"{ts},{id},{data}")

