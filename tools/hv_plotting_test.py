# -*- coding: utf-8 -*-
"""
Created on Sun Mar  1 08:17:38 2020

@author: benes
"""


# import time
# import numpy as np
# import pandas as pd
# import holoviews as hv
# import streamz
# import streamz.dataframe

# from holoviews import opts
# from holoviews.streams import Pipe, Buffer

# hv.extension('bokeh', logo=False)
# import panel as pn

# example = pd.DataFrame({'x': [], 'y': [], 'count': []}, columns=['x', 'y', 'count'])
# dfstream = Buffer(example, length=100, index=False)
# curve_dmap = hv.DynamicMap(hv.Curve, streams=[dfstream])
# point_dmap = hv.DynamicMap(hv.Points, streams=[dfstream])

# (curve_dmap * point_dmap).opts(
#     opts.Points(color='count', line_color='black', size=5, padding=0.1, xaxis=None, yaxis=None),
#     opts.Curve(line_width=1, color='black'))
# # display graph in browser
# # a bokeh server is automatically started
# bokeh_server = pn.Row(hv_plot).show(port=12345)
# def gen_brownian():
#     x, y, count = 0, 0, 0
#     while True:
#         x += np.random.randn()
#         y += np.random.randn()
#         count += 1
#         yield pd.DataFrame([(x, y, count)], columns=['x', 'y', 'count'])

# brownian = gen_brownian()
# for i in range(200):
#     dfstream.send(next(brownian))

    # library imports
import numpy as np
import pandas as pd
import holoviews as hv
hv.extension('bokeh')
import panel as pn

# create sample data
data = np.random.normal(size=[50, 2])
df = pd.DataFrame(data, columns=['col1', 'col2'])

# create holoviews graph
hv_plot = hv.Points(df)

# display graph in browser
# a bokeh server is automatically started
bokeh_server = pn.Row(hv_plot).show(port=12345)

# stop the bokeh server (when needed)
bokeh_server.stop()