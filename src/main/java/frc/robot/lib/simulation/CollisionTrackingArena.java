package frc.robot.lib.simulation;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.contact.Contact;
import org.dyn4j.world.ContactCollisionData;
import org.dyn4j.world.listener.ContactListenerAdapter;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

/** Counts actual dyn4j robot contacts with static field geometry, including walls and trenches. */
public class CollisionTrackingArena extends Arena2026Rebuilt {
  private int fieldContacts;

  public CollisionTrackingArena(SwerveDriveSimulation robot) {
    super(false); // Bumps use RobotBumpSim; retain the solid field obstacles.
    physicsWorld.addContactListener(
        new ContactListenerAdapter<Body>() {
          @Override
          public void begin(ContactCollisionData<Body> collision, Contact contact) {
            var first = collision.getBody1();
            var second = collision.getBody2();
            if ((first == robot && second.isStatic()) || (second == robot && first.isStatic()))
              fieldContacts++;
          }
        });
    addDriveTrainSimulation(robot);
  }

  public int getFieldContacts() {
    return fieldContacts;
  }
}
