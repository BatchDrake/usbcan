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

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as pch
import matplotlib.ticker as ticker
import pandas as pd

class CANAnalyzer:
  def __init__(self, path: str):
    df = pd.read_csv(path, names = ['timestamp', 'id', 'data'])
    self._streams = {}
    self._ts      = {}

    for i in range(len(df['id'])):
      can_id = df['id'][i]
      if not can_id in self._streams:
        self._streams[can_id] = []
        self._ts[can_id]      = []

      self._streams[can_id].append(bytearray.fromhex(df['data'][i]))
      self._ts[can_id].append(df['timestamp'][i])

  def streams(self):
    return list(self._streams.keys())

  def get_stream(self, can_id: int):
    if not can_id in self._streams:
      raise RuntimeError(fr'Stream {can_id} not found in this capture')

    return self._streams[can_id], self._ts[can_id]

  @staticmethod
  def paint_range(name, start, end, length, color = 'red', ax = None):
    rect = pch.Rectangle((-0.5, start), length, end - start, linewidth=1, edgecolor=color, facecolor=color, alpha = .0625, zorder = -4)
    if ax is None:
        ax = plt.gca()
        
    ax.add_patch(rect)
    ax.plot([-0.5, length - 1 + .5], [start, start], color = color, linestyle = 'dashed', linewidth = 1)
    ax.plot([-0.5, length - 1 + .5], [end,   end],   color = color, linestyle = 'dashed', linewidth = 1)

    ax.text((length - 1) / 2, start + (end - start) / 2, name, ha = 'center', va = 'center', c = color, alpha = .75)

  @staticmethod
  def to_hex(x, pos):
    return fr'0x{int(x):x}'

  def show_statistics(self, stream_id):
    stream_data, ts = self.get_stream(stream_id)
    bytematrix      = np.array(stream_data)
    length          = bytematrix.shape[1]

    if length == 0:
        print('Packets from this stream are 0-length')
        return
        
    bb              = np.arange(length)
    
    # Calculate ranges
    bMu  = np.mean(bytematrix, axis = 0)
    bmax = np.max(bytematrix,  axis = 0)
    bmin = np.min(bytematrix,  axis = 0)


    fig, ax = plt.subplots(1, 2, figsize=(14, 4.5))
    
    ax[0].scatter(bb, bMu, color = 'red')
    ax[0].bar(bb, height = bmax - bmin + .2, bottom = bmin - 0.1, linewidth = 3, zorder = -3, color = 'red', alpha = .25)
    fmt = ticker.FuncFormatter(self.to_hex)
    ax[0].get_yaxis().set_major_formatter(fmt)
    ax[0].set_xlim([-.5, length - 1 + .5])
    self.paint_range('Numerals', 0x30, 0x39, length, 'green', ax = ax[0])
    self.paint_range('ASCII (upper)',  0x41, 0x5a, length, 'green', ax = ax[0])
    self.paint_range('ASCII (lower)',  0x61, 0x7a, length, 'green', ax = ax[0])
    self.paint_range('',      0x20, 0x7e, length, 'gray', ax = ax[0])

    Ptrans = np.sum(
      np.diff(bytematrix, axis = 0) != 0, 
      axis = 0) / bytematrix.shape[0]

    ax[1].bar(bb, Ptrans, zorder = 10)
    ax[1].set_xlim([-.5, length - 1 + .5])
    ax[1].set_yscale('log')
    ax[1].grid()

    ax[0].set_xlabel('Offset')
    ax[0].set_ylabel('Byte')
    ax[0].set_title('Byte range')
    
    ax[1].set_xlabel('Offset')
    ax[1].set_ylabel('$P_t$')
    ax[1].set_title('Transition probability')

    fig.tight_layout()
    fig.suptitle(fr'CAN_ID = 0x{stream_id:x} ({length} bytes / frame)')
  
  def get_frame_template(self, stream_id):
    data, _  = self.get_stream(stream_id)
    arr      = np.array(data)
    length   = arr.shape[1]
    template = ''
    
    if length > 0:
        bMax = np.max(arr, axis = 0)
        bMin = np.min(arr, axis = 0)
        eq   = bMax == bMin
        for i in range(length):
            if eq[i]:
                template += fr'{bMax[i]:02x}'
            else:
                template += '??'
    return template

  def show_time_series(self, stream_id, offsets = None, ax = None):
    data, ts = self.get_stream(stream_id)
    arr      = np.array(data)
    length   = arr.shape[1]


    if offsets is None:
      bMax = np.max(arr, axis = 0)
      bMin = np.min(arr, axis = 0)

      where = np.where(bMax > bMin)[0]
      if where.shape[0] > 0:
        if ax is None:
          plt.figure(figsize=(14, 4.5))
          ax = plt.gca()

          for i in range(where.shape[0]):
            ax.plot(ts - ts[0], arr[:, where[i]], label = fr'Byte at offset {where[i]}', alpha = .25)
            ax.scatter(ts - ts[0], arr[:, where[i]], 4)
            ax.legend(loc = 'upper right')
          fmt = ticker.FuncFormatter(self.to_hex)
          ax.get_yaxis().set_major_formatter(fmt)
          ax.set_ylabel('Byte value')
      else:
        print(fr'CAN_ID = 0x{stream_id:x}: Stream does not evolve with time.')
        return None
    else:
      field    = np.zeros(arr.shape[0])
      for i in range(len(offsets) - 1, -1, -1):
        if offsets[i] < 0 or offsets[i] >= length:
          raise RuntimeError(fr'Offset `{offsets[i]} at index {i} is out of bounds for {length}-sized frames')
        field += arr[:, offsets[i]] * (1 << i * 8)
        
      if ax is None:
        plt.figure(figsize=(14, 4.5))
        ax = plt.gca()
      ax.plot(ts - ts[0], field)
      ax.scatter(ts - ts[0], field, 4)
      ax.set_ylabel('Value')
    
    ax.set_xlim([0, ts[-1] - ts[0]])
    ax.set_xlabel('Time since start [s]')
    
    ax.grid()
    
    return ax
