import 'package:flutter/material.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/constants/constants.dart';
import 'package:web_interface_flutter/services/api_service.dart';
import 'package:web_interface_flutter/widgets/rounded_button.dart';

class ResumeButton extends StatelessWidget {
  const ResumeButton({super.key});

  @override
  Widget build(BuildContext context) {
    return RoundedButton(
      text: "Resume",
      padding: Constants.largePadding,
      color: AppColors.blue,
      onTap: () async {
        APIService.pauseOrResumeRobot(robotName: "ROBAST", fleetName: "RB0", shouldPause: false);
      },
    );
  }
}
