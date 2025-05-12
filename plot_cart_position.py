import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv('cart_pole_data.csv')

plt.figure(figsize=(10,5))
plt.plot(data['Time'], data['Cart Position'], label='Cart Position')
plt.axhline(2.5, color='r', linestyle='--', label='Upper Limit 2.5m')
plt.axhline(-1, color='g', linestyle='--', label='Lower Limit -1m')
plt.xlim(0, 300)
plt.xlabel('Time (s)')
plt.ylabel('Cart Position (m)')
plt.title('Cart Position Over Time')
plt.legend()
plt.grid(True)
plt.show()

