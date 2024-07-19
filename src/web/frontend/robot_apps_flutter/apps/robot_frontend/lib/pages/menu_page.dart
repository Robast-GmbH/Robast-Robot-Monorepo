import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';
import 'package:robot_frontend/pages/module_filling_page.dart';
import 'package:robot_frontend/pages/settings_page.dart';
import 'package:robot_frontend/pages/task_creation_menu_page.dart';
import 'package:robot_frontend/pages/tasks_overview_page.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class MenuPage extends StatelessWidget {
  const MenuPage({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      onBackButtonPressed: () {
        Provider.of<RobotProvider>(context, listen: false).unblockNavigation();
        Provider.of<UserProvider>(context, listen: false).endUserSession(robotName: 'rb_theron');
        Navigator.pop(context);
        Navigator.pop(context);
      },
      title: 'Hauptmenü',
      child: Padding(
        padding: const EdgeInsets.all(128),
        child: Row(
          children: [
            Expanded(
              child: Column(
                children: [
                  Expanded(
                    child: CustomButtonView(
                      text: 'Auftrag erstellen',
                      onPressed: () => Navigator.push(
                        context,
                        MaterialPageRoute<TaskCreationMenuPage>(
                          builder: (context) => const TaskCreationMenuPage(),
                        ),
                      ),
                    ),
                  ),
                  Expanded(
                    child: CustomButtonView(
                      text: 'Auftragsübersicht',
                      onPressed: () => Navigator.push(
                        context,
                        MaterialPageRoute<TasksOverviewPage>(
                          builder: (context) => const TasksOverviewPage(),
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ),
            Expanded(
              child: Column(
                children: [
                  Expanded(
                    child: CustomButtonView(
                      text: 'Module befüllen',
                      onPressed: () => Navigator.push(
                        context,
                        MaterialPageRoute<ModuleFillingPage>(
                          builder: (context) => const ModuleFillingPage(),
                        ),
                      ),
                    ),
                  ),
                  Expanded(
                    child: CustomButtonView(
                      text: 'Einstellungen',
                      onPressed: () => Navigator.push(
                        context,
                        MaterialPageRoute<SettingsPage>(
                          builder: (context) => const SettingsPage(),
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
