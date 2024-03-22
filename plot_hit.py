import matplotlib.pyplot as plt
import numpy as np

scenarios = ("Env-A", "Env-B", "Env-C", "Env-D", "Env-E")
methods = {
    'ORCA': (74, 37, 63, 22, 0.5),
    'CBF': (92, 58, 76, 31, 0.5),
    'RRT with MP': (100, 100, 100, 100, 100),
}

x = np.arange(len(scenarios))  # the label locations
width = 0.20  # the width of the bars
multiplier = 0

fig, ax = plt.subplots(layout='constrained')

for attribute, measurement in methods.items():
    offset = width * multiplier
    rects = ax.bar(x + offset, measurement, width, label=attribute)
    # ax.bar_label(rects, padding=3)
    multiplier += 1

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Success Rate (%)')
ax.set_title('Methods comparison in different environments')
ax.set_xticks(x + width, scenarios)
ax.legend(loc='upper left', ncols=3)
ax.set_ylim(0, 110)

plt.show()