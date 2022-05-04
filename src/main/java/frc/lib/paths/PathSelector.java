package frc.lib.paths;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class PathSelector {
    SendableChooser<Path> chooser;
    Map<String,Path> paths;

    /**
     * Handles choosing Paths in a dropbox in the Shuffleboard UI.
     * @param paths - List of the Paths in the drop-box
     * @param name - Shuffleboard Tab name
     */
    public PathSelector(Map<String,Path> paths,String name){
        this(paths, Shuffleboard.getTab(name));
    }

    /**
     * Handles choosing Paths in a dropbox in the Shuffleboard UI.
     * @param paths - List of the Paths in the drop-box
     * @param tab - Shuffleboard Tab to put the dropbox in.
     */
    public PathSelector(Map<String,Path> paths,ShuffleboardTab tab) {
        List<Entry<String, Path>> pathList = new ArrayList<>();
        this.paths = paths;
        chooser = new SendableChooser<>();
        
        for(Entry<String,Path> e : paths.entrySet()){
            pathList.add(e);
        }
        
        chooser.setDefaultOption(pathList.get(0).getKey(), pathList.get(0).getValue());
        for(int i = 1; i < pathList.size();i++){
            chooser.addOption(pathList.get(i).getKey(), pathList.get(i).getValue());
        }

        tab.add(chooser);
    }

    public Path getPath() {
        return chooser.getSelected();
    }
}
