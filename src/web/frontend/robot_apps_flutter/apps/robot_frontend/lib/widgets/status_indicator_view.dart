import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/pages/config_page.dart';
import 'package:robot_frontend/widgets/developer_button_view.dart';

class StatusIndicatorView extends StatelessWidget {
  const StatusIndicatorView({super.key});

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.end,
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        const Icon(
          Icons.icecream,
          size: 40,
        ),
        const SizedBox(width: 8),
        const Icon(
          Icons.link,
          size: 40,
        ),
        const SizedBox(width: 8),
        const RotatedBox(
          quarterTurns: 1,
          child: Icon(
            Icons.battery_5_bar,
            size: 40,
          ),
        ),
        const SizedBox(width: 8),
        //RobotLostIndicator(),
        DeveloperButtonView(
          onPressed: () {
            Navigator.push(context, MaterialPageRoute<ConfigPage>(builder: (context) => const ConfigPage()));
          },
          child: Container(
            margin: const EdgeInsets.all(4),
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: const Color.fromARGB(255, 0, 226, 0),
              border: Border.all(
                color: RobotColors.primaryText,
                width: 4,
              ),
            ),
            width: 30,
            height: 30,
          ),
        ),
      ],
    );
  }
}
