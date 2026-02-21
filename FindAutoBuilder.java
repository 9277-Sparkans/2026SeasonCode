import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import java.lang.reflect.Method;
import java.util.function.Supplier;

public class FindAutoBuilder {
    public static void main(String[] args) {
        for (Method m : AutoBuilder.class.getDeclaredMethods()) {
            if (m.getName().contains("pathfind")) {
                System.out.println(m);
            }
        }
    }
}
