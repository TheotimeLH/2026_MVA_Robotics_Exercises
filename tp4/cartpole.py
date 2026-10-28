"""
A toy cart-pole model built from scratch with Pinocchio.

Kinematic tree:
    universe --(prismatic X: "cart")--> cart body
             --(revolute Y: "pole")--> pole body (uniform rod)

Configuration q = [p, theta]:
    p     : cart position along the rail (m)
    theta : pole angle (rad). theta = 0 is the pole pointing UP (unstable
            equilibrium), theta = pi is the pole hanging DOWN (stable).
Velocity v = [p_dot, theta_dot]. Both joints are Euclidean (nq = nv = 2).
Only the cart is actuated: tau = [u, 0].
"""

import time

import coal
import meshcat
import numpy as np
import pinocchio as pin


def build_cartpole(cart_mass=1.0, pole_mass=0.3, pole_length=0.8, rail_length=4.0):
    """Return a pin.RobotWrapper for the cart-pole (model, collision and visual models)."""
    model = pin.Model()
    model.name = "cartpole"
    geom_model = pin.GeometryModel()

    cart_size = np.array([0.3, 0.2, 0.15])
    pole_radius = 0.02

    # --- Cart: prismatic joint along the world X axis.
    cart_joint = model.addJoint(0, pin.JointModelPX(), pin.SE3.Identity(), "cart")
    cart_joint_frame = model.addJointFrame(cart_joint, 0)
    model.appendBodyToJoint(
        cart_joint, pin.Inertia.FromBox(cart_mass, *cart_size), pin.SE3.Identity()
    )
    model.addBodyFrame("cart_body", cart_joint, pin.SE3.Identity(), cart_joint_frame)

    # --- Pole: revolute joint around Y, located at the cart center.
    # The rod is attached at its bottom end, its center of mass is l/2 above the pivot.
    pole_joint = model.addJoint(cart_joint, pin.JointModelRY(), pin.SE3.Identity(), "pole")
    pole_joint_frame = model.addJointFrame(pole_joint, cart_joint_frame)
    com = pin.SE3(np.eye(3), np.array([0.0, 0.0, pole_length / 2]))
    model.appendBodyToJoint(
        pole_joint, pin.Inertia.FromCylinder(pole_mass, pole_radius, pole_length), com
    )
    model.addBodyFrame("pole_body", pole_joint, com, pole_joint_frame)
    tip = pin.SE3(np.eye(3), np.array([0.0, 0.0, pole_length]))
    model.addFrame(pin.Frame("pole_tip", pole_joint, pole_joint_frame, tip, pin.FrameType.OP_FRAME))

    # --- Geometries (for display only).
    def add_geom(name, joint, placement, shape, color):
        geom = pin.GeometryObject(name, joint, placement, shape)
        geom.meshColor = np.array(color)
        geom_model.addGeometryObject(geom)

    add_geom("rail", 0, pin.SE3(np.eye(3), np.array([0.0, 0.0, -cart_size[2] / 2 - 0.01])),
             coal.Box(rail_length, 0.04, 0.02), [0.4, 0.4, 0.4, 1.0])
    add_geom("cart", cart_joint, pin.SE3.Identity(), coal.Box(*cart_size), [0.2, 0.4, 0.8, 1.0])
    add_geom("pole", pole_joint, com, coal.Cylinder(pole_radius, pole_length), [0.9, 0.6, 0.1, 1.0])
    add_geom("tip", pole_joint, tip, coal.Sphere(0.05), [0.8, 0.1, 0.1, 1.0])

    # The same simple shapes are used for collision and display.
    return pin.RobotWrapper(model, geom_model, geom_model)


def set_side_view(viz, distance=4.0, height=0.5):
    """Place the Meshcat camera on the -Y side, looking at the (X, Z) plane of motion."""
    # The meshcat camera lives in a y-up frame: local (x, y, z) -> world (x, -z, y).
    viz.viewer["/Cameras/default"].set_transform(np.eye(4))
    viz.viewer["/Cameras/default/rotated/<object>"].set_property("position", [0.0, height, distance])


def add_target(viz, color=(0.1, 0.8, 0.1, 0.4)):
    """Show a transparent green ball where the pole tip should be: cart at 0, pole up."""
    tip = viz.model.frames[viz.model.getFrameId("pole_tip")].placement.translation
    viz.viewer["target"].set_object(
        meshcat.geometry.Sphere(0.07),
        meshcat.geometry.MeshLambertMaterial(color=rgb_to_int(color), transparent=True, opacity=color[3]),
    )
    viz.viewer["target"].set_transform(meshcat.transformations.translation_matrix([0.0, 0.0, tip[2]]))


def rgb_to_int(color):
    r, g, b = (int(255 * c) for c in color[:3])
    return (r << 16) + (g << 8) + b


def play_trajectory(viz, X, dt, speed=1.0, trace=True):
    """Display a state trajectory X (array of shape (N+1, nq+nv)) in real time.

    If trace is True, the path of the pole tip is drawn as the motion goes.
    """
    nq = viz.model.nq
    tip_id = viz.model.getFrameId("pole_tip")
    viz.viewer["trace"].delete()
    tips = []
    for x in X:
        t0 = time.time()
        viz.display(x[:nq])
        if trace:
            pin.updateFramePlacement(viz.model, viz.data, tip_id)
            tips.append(viz.data.oMf[tip_id].translation.copy())
            if len(tips) > 1:
                viz.viewer["trace"].set_object(
                    meshcat.geometry.Line(
                        meshcat.geometry.PointsGeometry(np.array(tips).T),
                        meshcat.geometry.LineBasicMaterial(color=0xCC3311, linewidth=2),
                    )
                )
        time.sleep(max(0.0, dt / speed - (time.time() - t0)))


def plot_trajectory(X, dt, U=None, title=""):
    """Plot the state trajectory X (N+1, 4) and optionally the controls U (N, 1)."""
    import matplotlib.pyplot as plt

    n_plots = 3 if U is not None else 2
    fig, axs = plt.subplots(1, n_plots, figsize=(5 * n_plots, 3))
    t = np.arange(len(X)) * dt
    axs[0].plot(t, X[:, 0], label=r"$p$ (m)")
    axs[0].plot(t, X[:, 1], label=r"$\theta$ (rad)")
    axs[0].set_title("configuration")
    axs[1].plot(t, X[:, 2], label=r"$\dot p$ (m/s)")
    axs[1].plot(t, X[:, 3], label=r"$\dot \theta$ (rad/s)")
    axs[1].set_title("velocity")
    if U is not None:
        axs[2].stairs(np.ravel(U), t, baseline=None, label=r"$u$ (N)")
        axs[2].set_title("control")
    for ax in axs:
        ax.set_xlabel("time (s)")
        ax.legend()
        ax.grid(True)
    fig.suptitle(title)
    plt.tight_layout()
    plt.show()
