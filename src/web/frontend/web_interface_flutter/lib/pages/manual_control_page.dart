import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/constants/constants.dart';
import 'package:web_interface_flutter/models/data/drawer_module.dart';
import 'package:web_interface_flutter/models/provider/robot_provider.dart';
import 'package:web_interface_flutter/services/api_service.dart';
import 'package:web_interface_flutter/widgets/page_frame.dart';
import 'package:web_interface_flutter/widgets/robot_clone.dart';

class ManualControlPage extends StatefulWidget {
  const ManualControlPage({super.key});

  @override
  State<ManualControlPage> createState() => _ManualControlPageState();
}

class _ManualControlPageState extends State<ManualControlPage> {
  @override
  void initState() {
    super.initState();
    Provider.of<RobotProvider>(context, listen: false).startPeriodicModuleUpdate();
  }

  @override
  void deactivate() {
    Provider.of<RobotProvider>(context, listen: false).stopPeriodicModuleUpdate();
    super.deactivate();
  }

  @override
  Widget build(BuildContext context) {
    final robotProvider = Provider.of<RobotProvider>(context, listen: false);
    return PageFrame(
      color: AppColors.blue,
      title: "Freie Schubladenauswahl",
      child: Container(
        margin: Constants.mediumPadding,
        decoration: BoxDecoration(
          color: AppColors.lightGrey,
          borderRadius: BorderRadius.circular(16),
        ),
        child: Stack(
          children: [
            Center(
              child: Padding(
                padding: Constants.largePadding,
                child: RobotClone(
                  displayStatus: true,
                  onPressed: (moduleID) async {
                    try {
                      
                      final module = robotProvider.modules["rb_theron"]?.firstWhere((element) => element.moduleID == moduleID);
                      if (module!.status == "Opened" && module.type == ModuleType.electric_drawer) {
                        await APIService.closeDrawer(robotName: "rb_theron", moduleID: moduleID, drawerID: module.drawerID);
                      } else if (module.status == "Closed") {
                        await APIService.openDrawer(robotName: "rb_theron", moduleID: moduleID, drawerID: module.drawerID);
                      } else {
                        return;
                      }
                      await robotProvider.updateModules();
                    } catch (e) {
                      debugPrint(e.toString());
                    }
                  },
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
