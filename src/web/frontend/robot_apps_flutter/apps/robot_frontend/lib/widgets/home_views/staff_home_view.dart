import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/menu_modules_overview.dart';
import 'package:robot_frontend/widgets/menu_tasks_overview.dart';
import 'package:robot_frontend/widgets/task_creation_button_list.dart';
import 'package:robot_frontend/widgets/welcome_view.dart';

class StaffHomeView extends StatelessWidget {
  const StaffHomeView({super.key});

  @override
  Widget build(BuildContext context) {
    return const Row(
      children: [
        Expanded(
          flex: 2,
          child: Column(
            children: [
              Expanded(
                child: WelcomeView(),
              ),
              SizedBox(height: 16),
              Expanded(
                child: Row(
                  children: [
                    Expanded(
                      child: MenuTasksOverview(),
                    ),
                    SizedBox(
                      width: 16,
                    ),
                    Expanded(
                      child: TaskCreationButtonList(),
                    ),
                  ],
                ),
              ),
            ],
          ),
        ),
        SizedBox(width: 16),
        Expanded(
          child: Column(
            children: [
              Expanded(
                child: MenuModulesOverview(),
              ),
            ],
          ),
        ),
      ],
    );
  }
}
