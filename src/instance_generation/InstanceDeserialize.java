package instance_generation;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.ObjectInputStream;

public class InstanceDeserialize {
    public static Instance deserialize(String filename) throws IOException, ClassNotFoundException {
        ObjectInputStream objectInputStream =
                new ObjectInputStream( new FileInputStream( filename ) );
        Instance instance = (Instance) objectInputStream.readObject();
        objectInputStream.close();
        return instance;
    }
}
