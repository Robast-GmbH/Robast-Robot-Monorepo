import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';
import 'package:robot_frontend/widgets/centered_view.dart';
import 'package:shared_data_models/shared_data_models.dart';

class ClosedView extends StatelessWidget {
  const ClosedView({
    required this.onFinish,
    required this.onReopen,
    required this.submodule,
    required this.secondsToFinish,
    super.key,
  });
  final VoidCallback onFinish;
  final VoidCallback onReopen;
  final Submodule submodule;
  final int secondsToFinish;

  @override
  Widget build(BuildContext context) {
    return CenteredView(
      child: Padding(
        padding: const EdgeInsets.symmetric(vertical: 192),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Expanded(
              flex: 2,
              child: CustomButtonView(
                text: 'Fertigstellen',
                titleFontSize: 56,
                content: Text(
                  '(automatisch in ${5 - secondsToFinish} Sekunden)',
                  style: const TextStyle(color: RobotColors.secondaryText, fontSize: 32),
                ),
                onPressed: onFinish,
              ),
            ),
            const SizedBox(
              height: 8,
            ),
            Expanded(
              child: CustomButtonView(
                text: 'Erneut öffnen',
                titleFontSize: 56,
                onPressed: onReopen,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
