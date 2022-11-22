package frc.lib;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PrimoTab {
    private String title;
    private Map<String, NetworkTableEntry> entryMap;
    private ShuffleboardTab tab;
    public PrimoTab(String title){
        this.title = title;
        entryMap = new HashMap<>();
        tab = Shuffleboard.getTab(title);
    }

    public ShuffleboardTab getTab() {
        return tab;
    }

    public NetworkTableEntry addEntry(String name) {
        if (entryMap.containsKey(name)) {
            NetworkTableEntry entry = entryMap.get(name);
            return entry;
        }
        NetworkTableEntry entry = tab.add(name, 0).getEntry();
        entryMap.put(name, entry);
        return entry;
    }
    public String getTitle() {
        return title;
    }
    
}
