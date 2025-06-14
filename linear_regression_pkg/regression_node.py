import warnings
warnings.filterwarnings("ignore", message="Unable to import Axes3D")

import matplotlib
# Use non-GUI backend for saving plots in headless environments
matplotlib.use('Agg')  
import matplotlib.pyplot as plt

import os
import rclpy
from rclpy.node import Node

import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_absolute_error, r2_score
from sklearn.impute import SimpleImputer
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory

# Function to train model, plot predictions, and return evaluation metrics
def train_and_plot(X_train, y_train, X_test, y_test, title, save_path):
    model = LinearRegression()
    model.fit(X_train, y_train)
    y_pred = model.predict(X_test)

    mae = mean_absolute_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    # Create scatter plot for predicted
    plt.scatter(y_test, y_pred)
    plt.title(f"{title} – MAE: {mae:.2f}, R2: {r2:.2f}")
    plt.xlabel('Actual')
    plt.ylabel('Predicted')

    # Save the plot to disk
    filename = f"{title.replace(' ', '_')}.png"
    out_png = os.path.join(save_path, filename)
    plt.savefig(out_png)
    plt.clf()  # Clear current figure for the next one

    return mae, r2

# Main regression node class
class RegressionNode(Node):
    def __init__(self):
        super().__init__('regression_node')
        self.get_logger().info("Linear Regression Node Started.")

        # Create publisher for regression results
        self.pub = self.create_publisher(Float32MultiArray, 'regression_results', 10)

        # Get package path to access data and output folders
        self.package_path = get_package_share_directory('linear_regression_pkg')

        # Run each dataset processing function
        self.run_dataset_1()
        self.run_dataset_2()
        self.run_dataset_3()

    # Publishes result metrics (dataset ID, MAE, R2) to a topic
    def publish_metrics(self, dataset_id, mae, r2):
        msg = Float32MultiArray()
        msg.data = [float(dataset_id), float(mae), float(r2)]
        self.pub.publish(msg)

    # Process Dataset 1: Height and Weight
    def run_dataset_1(self):
        path = os.path.join(self.package_path, 'data', 'new_height_weight.csv')
        df = pd.read_csv(path)
        X = df[['Height']]
        y = df['Weight']
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2)
        mae, r2 = train_and_plot(X_train, y_train, X_test, y_test, 'Height vs Weight', self.package_path)
        self.get_logger().info(f"Dataset1 – MAE: {mae:.2f}, R2: {r2:.2f}")
        self.publish_metrics(1, mae, r2)

    # Process Dataset 2: Head Size and Brain Weight
    def run_dataset_2(self):
        path = os.path.join(self.package_path, 'data', 'HumanBrain_WeightandHead_size.csv')
        df = pd.read_csv(path)
        X = df[['Head Size(cm^3)']]
        y = df['Brain Weight(grams)']
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2)
        mae, r2 = train_and_plot(X_train, y_train, X_test, y_test, 'Head Size vs Brain Weight', self.package_path)
        self.get_logger().info(f"Dataset2 – MAE: {mae:.2f}, R2: {r2:.2f}")
        self.publish_metrics(2, mae, r2)

    # Process Dataset 3: Boston Housing
    def run_dataset_3(self):
        path = os.path.join(self.package_path, 'data', 'Boston_Housing.csv')
        df = pd.read_csv(path)

        # Normalize column names
        df.columns = df.columns.str.strip().str.lower()
        X = df.drop(columns=['medv'])
        y = df['medv']

        # Replace missing values with column means
        imputer = SimpleImputer(strategy='mean')
        X = pd.DataFrame(imputer.fit_transform(X), columns=X.columns)

        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2)
        mae, r2 = train_and_plot(X_train, y_train, X_test, y_test, 'Boston Housing', self.package_path)
        self.get_logger().info(f"Dataset3 – MAE: {mae:.2f}, R2: {r2:.2f}")
        self.publish_metrics(3, mae, r2)

# Main entry point
def main(args=None):
    rclpy.init(args=args)
    node = RegressionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
