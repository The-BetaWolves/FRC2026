package frc.robot.services;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;

public class QualityControlService {
    private final ShuffleboardTab tab;
    private final int startCol;
    private final int startRow;
    private final int widgetWidth = 3;
    private final int widgetHeight = 2;

    public QualityControlService(String title, int startCol, int startRow) {

        this.tab = Shuffleboard.getTab("Quality Control");
        this.startCol = startCol;
        this.startRow = startRow;

        // Header for this section
        this.tab.add(title, " ")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(startCol, startRow)
            .withSize(widgetWidth, widgetHeight);
    }

    private int nextRowOffset = 1; // row 0 is the header

    // setup a new monitored hardware to watch
    public MonitoredHardware watch(String name) {

        // keep track of where new items go on the tab grid
        int row = startRow + nextRowOffset;
        nextRowOffset += widgetHeight;

        // add the entry to the tab grid
        GenericEntry entry = tab.add(name, false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(startCol, row)
            .withSize(widgetWidth, widgetHeight)
            .getEntry();
            return new MonitoredHardware(entry);
    }

    public static class MonitoredHardware {
        private final GenericEntry poweredEntry;

        private MonitoredHardware(GenericEntry poweredEntry) {
            this.poweredEntry = poweredEntry;
        }

        // update the item on the tab grid
        public void update(boolean powered) {
            poweredEntry.setBoolean(powered);
        }
    }
}

