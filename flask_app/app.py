from flask import Flask, render_template, request, jsonify
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import base64
import io
import json
from math import comb

# Import our Bezier curve classes
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src', 'main'))

# Copy the classes from the original file
class Point:
    """A simple 2D point class to mimic the Point class in Java"""
    def __init__(self, x, y=None):
        if y is None and isinstance(x, (list, tuple, np.ndarray)):
            self.x = float(x[0])
            self.y = float(x[1])
        else:
            self.x = float(x)
            self.y = float(y)
    
    def distance(self, other):
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def to_array(self):
        return np.array([self.x, self.y])
    
    def __str__(self):
        return f"Point({self.x}, {self.y})"

class Vector:
    """A simple 2D vector class to mimic the Vector class in Java"""
    def __init__(self, x, y=None):
        if y is None and isinstance(x, (list, tuple, np.ndarray)):
            self.x = float(x[0])
            self.y = float(x[1])
        else:
            self.x = float(x)
            self.y = float(y)
    
    def dot(self, other):
        return self.x * other.x + self.y * other.y
    
    def cross(self, other):
        return self.x * other.y - self.y * other.x
    
    def magnitude(self):
        return np.sqrt(self.x**2 + self.y**2)
    
    def to_array(self):
        return np.array([self.x, self.y])
    
    def __str__(self):
        return f"Vector({self.x}, {self.y})"

class BezierCurve:
    """Base class for Bezier curves"""
    def __init__(self, control_points=None):
        if control_points is None:
            self.control_points = []
        elif isinstance(control_points[0], Point):
            self.control_points = control_points
        else:
            self.control_points = [Point(p) for p in control_points]
        
        if len(self.control_points) < 3:
            print("Warning: Bezier curve should have at least 3 control points")
    
    def get_point(self, t):
        """Calculate point on the Bezier curve at parameter t"""
        n = len(self.control_points) - 1
        result = Point(0, 0)
        
        for i, point in enumerate(self.control_points):
            coefficient = comb(n, i) * ((1 - t) ** (n - i)) * (t ** i)
            result.x += coefficient * point.x
            result.y += coefficient * point.y
            
        return result
    
    def get_derivative(self, t):
        """Calculate first derivative at parameter t"""
        n = len(self.control_points) - 1
        result = Vector(0, 0)
        
        for i in range(n):
            coefficient = n * comb(n - 1, i) * ((1 - t) ** (n - 1 - i)) * (t ** i)
            p1 = self.control_points[i + 1]
            p0 = self.control_points[i]
            result.x += coefficient * (p1.x - p0.x)
            result.y += coefficient * (p1.y - p0.y)
            
        return result
    
    def get_second_derivative(self, t):
        """Calculate second derivative at parameter t"""
        n = len(self.control_points) - 1
        if n < 2:
            return Vector(0, 0)
            
        result = Vector(0, 0)
        
        for i in range(n - 1):
            coefficient = n * (n - 1) * comb(n - 2, i) * ((1 - t) ** (n - 2 - i)) * (t ** i)
            p2 = self.control_points[i + 2]
            p1 = self.control_points[i + 1]
            p0 = self.control_points[i]
            result.x += coefficient * (p2.x - 2 * p1.x + p0.x)
            result.y += coefficient * (p2.y - 2 * p1.y + p0.y)
            
        return result
    
    def get_curvature(self, t):
        """Calculate curvature at parameter t"""
        d1 = self.get_derivative(t)
        d2 = self.get_second_derivative(t)
        
        # Cross product in 2D
        cross = d1.cross(d2)
        
        # Avoid division by zero
        denominator = d1.magnitude() ** 3
        if denominator < 1e-8:
            denominator = 1e-8
            
        return abs(cross) / denominator

class OptimisedBezierCurve(BezierCurve):
    """Optimized Bezier curve that avoids obstacles"""
    def __init__(self, control_points, penalty_multiplier=1.0, obstacle_points=None, robot_mass=1.0):
        super().__init__(control_points)
        
        self.penalty_multiplier = penalty_multiplier
        self.obstacle_points = [] if obstacle_points is None else obstacle_points
        self.robot_mass = robot_mass
        
        # Store original control points for shape penalty calculation
        if isinstance(control_points[0], Point):
            self.original_control_points = control_points.copy()
        else:
            self.original_control_points = [Point(p) for p in control_points]
        
        # Optimize the control points
        self.optimize_bezier_curve()
    
    def potential_function(self, point, strength=1000, radius=9):
        """Calculate potential field at a point due to obstacles"""
        potential = 0
        for obs in self.obstacle_points:
            if isinstance(obs, Point):
                dist = np.linalg.norm(np.array([point.x, point.y]) - np.array([obs.x, obs.y]))
            else:
                dist = point.distance(Point(obs))
                
            if dist < radius:
                potential += strength ** 10
                
        return potential
    
    def control_points_to_array(self):
        """Convert control points to a flat array for optimization"""
        # Only optimize interior control points (keep first and last fixed)
        result = []
        for i in range(1, len(self.control_points) - 1):
            result.extend([self.control_points[i].x, self.control_points[i].y])
        return np.array(result)
    
    def array_to_control_points(self, flat_array):
        """Convert flat array back to control points"""
        points = [self.control_points[0]]  # Keep first point
        
        for i in range(0, len(flat_array), 2):
            points.append(Point(flat_array[i], flat_array[i + 1]))
            
        points.append(self.control_points[-1])  # Keep last point
        return points
    
    def cost_function(self, params):
        """Cost function for optimization"""
        from scipy.optimize import minimize
        
        # Reconstruct control points from parameters
        temp_points = self.array_to_control_points(params)
        temp_curve = BezierCurve(temp_points)
        
        # Physical constraint penalties
        cost = 0
        shape_penalty = 0
        N = 10000  # Reduced for web performance
        
        for i in range(N):
            t = i / (N - 1)
            dt = 1/(N-1)
            
            # Get derivatives and curvature
            d1 = temp_curve.get_derivative(t)
            d2 = temp_curve.get_second_derivative(t)
            curvature = temp_curve.get_curvature(t)
            
            # Kinetic energy term
            kinetic_energy = (d1.dot(d1)**2)/2
            
            # Sample point for potential field
            sample_point = temp_curve.get_point(t)
            
            # Potential energy term
            potential_energy = self.potential_function(sample_point)
            
            # Lagrangian = T - V
            cost += (kinetic_energy + potential_energy)*dt
            
        # Combine all penalties with appropriate weights
        return cost + curvature**2
    
    def optimize_bezier_curve(self):
        """Optimize the control points to minimize the cost function"""
        from scipy.optimize import minimize
        
        # Convert control points to flat array for optimization
        initial_params = self.control_points_to_array()
        
        try:
            # Use Nelder-Mead optimization
            result = minimize(
                self.cost_function,
                initial_params,
                method='Nelder-Mead',
                options={'maxiter': 10000}  # Reduced for web performance
            )
            
            # Update control points with optimized ones
            if result.success:
                optimized_points = self.array_to_control_points(result.x)
                self.control_points = optimized_points
            else:
                print("Optimization failed:", result.message)
                
        except Exception as e:
            print(f"Error during optimization: {e}")

app = Flask(__name__)

def generate_plot(control_points, obstacles, penalty_multiplier=1.0, robot_mass=1.0):
    """Generate a plot and return it as a base64 encoded string"""
    # Create and optimize the curve
    curve = OptimisedBezierCurve(
        control_points=control_points,
        penalty_multiplier=penalty_multiplier,
        obstacle_points=obstacles,
        robot_mass=robot_mass
    )
    
    # Generate plot
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(12, 8))
    
    ts = np.linspace(0, 1, 100)
    
    # Get points along the optimized curve
    optimized_points = [curve.get_point(t) for t in ts]
    optimized_x = [p.x for p in optimized_points]
    optimized_y = [p.y for p in optimized_points]
    
    # Plot optimized curve
    ax.plot(optimized_x, optimized_y, '#00ff88', linewidth=3, label='Optimized Curve', alpha=0.9)
    
    # Plot optimized control points
    cp_x = [p.x for p in curve.control_points]
    cp_y = [p.y for p in curve.control_points]
    ax.plot(cp_x, cp_y, 'o-', color='#00ff88', alpha=0.7, markersize=8, linewidth=2, label='Optimized Control Points')
    
    # Create a temporary curve with original control points
    original_curve = BezierCurve(curve.original_control_points)
    original_points = [original_curve.get_point(t) for t in ts]
    original_x = [p.x for p in original_points]
    original_y = [p.y for p in original_points]
    
    # Plot original curve
    ax.plot(original_x, original_y, '#ff4444', linewidth=2, linestyle='--', alpha=0.8, label='Original Curve')
    
    # Plot original control points
    orig_cp_x = [p.x for p in curve.original_control_points]
    orig_cp_y = [p.y for p in curve.original_control_points]
    ax.plot(orig_cp_x, orig_cp_y, 'o-', color='#ff4444', alpha=0.6, markersize=6, linewidth=1.5, label='Original Control Points')
    
    # Plot obstacles
    for obs in obstacles:
        if isinstance(obs, Point):
            x, y = obs.x, obs.y
        else:
            x, y = obs[0], obs[1]
            
        circle = plt.Circle((x, y), 10, color='#ffaa00', alpha=0.6)
        ax.add_patch(circle)
        ax.plot(x, y, 'o', color='#ffaa00', markersize=8)
    
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', framealpha=0.9)
    ax.set_title('Optimised Bezier Curve Visualization', fontsize=16, color='white', pad=20)
    ax.set_xlabel('X Coordinate', fontsize=12, color='white')
    ax.set_ylabel('Y Coordinate', fontsize=12, color='white')
    
    # Style the plot
    ax.tick_params(colors='white')
    fig.patch.set_facecolor('#1a1a1a')
    ax.set_facecolor('#2d2d2d')
    
    # Convert plot to base64 string
    img = io.BytesIO()
    plt.savefig(img, format='png', bbox_inches='tight', facecolor='#1a1a1a', dpi=150)
    img.seek(0)
    plot_url = base64.b64encode(img.getvalue()).decode()
    
    plt.close(fig)  # Important: close the figure to free memory
    
    return plot_url

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/generate_curve', methods=['POST'])
def generate_curve():
    try:
        data = request.json
        
        # Parse control points
        control_points = []
        for point in data['controlPoints']:
            control_points.append([float(point['x']), float(point['y'])])
        
        # Parse obstacles
        obstacles = []
        for obs in data['obstacles']:
            obstacles.append([float(obs['x']), float(obs['y'])])
        
        # Use default parameters
        penalty_multiplier = 1.0
        robot_mass = 1.0
        
        # Generate plot
        plot_url = generate_plot(control_points, obstacles, penalty_multiplier, robot_mass)
        
        return jsonify({
            'success': True,
            'plot': plot_url
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 400

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5001)
