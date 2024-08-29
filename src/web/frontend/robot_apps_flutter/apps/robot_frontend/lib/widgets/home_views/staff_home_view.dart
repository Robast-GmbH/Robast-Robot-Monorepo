import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/menu_modules_overview.dart';
import 'package:robot_frontend/widgets/menu_tasks_overview.dart';
import 'package:robot_frontend/widgets/task_creation_button_list.dart';
import 'package:robot_frontend/widgets/welcome_view.dart';

class StaffHomeView extends StatelessWidget {
  const StaffHomeView({super.key});

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Expanded(
          flex: 2,
          child: Column(
            children: [
              const Expanded(
                child: WelcomeView(),
              ),
              const SizedBox(height: 16),
              Expanded(
                child: Row(
                  children: [
                    const Expanded(
                      child: MenuTasksOverview(),
                    ),
                    const SizedBox(
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
        const SizedBox(width: 16),
        const Expanded(
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
