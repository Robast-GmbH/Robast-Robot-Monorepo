import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';

class DeliveryPlanView extends StatelessWidget {
  const DeliveryPlanView({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomButtonView(
      text: 'Lieferplan',
      onPressed: () {},
      content: const Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Expanded(
                  child: Text(
                'Abendessen:',
                style: TextStyle(color: RobotColors.secondaryText, fontSize: 24),
              )),
              Expanded(
                  child: Text(
                'Lasagne',
                textAlign: TextAlign.start,
                style: TextStyle(color: RobotColors.secondaryText, fontSize: 24),
              )),
              Expanded(
                  child: Text(
                '17:00 Uhr',
                textAlign: TextAlign.end,
                style: TextStyle(color: RobotColors.secondaryText, fontSize: 24),
              )),
            ],
          ),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Expanded(
                  child: Text(
                'Medikament:',
                style: TextStyle(color: RobotColors.secondaryText, fontSize: 24),
              )),
              Expanded(
                  child: Text(
                'Aspirin',
                textAlign: TextAlign.start,
                style: TextStyle(color: RobotColors.secondaryText, fontSize: 24),
              )),
              Expanded(
                  child: Text(
                '19:00 Uhr',
                textAlign: TextAlign.end,
                style: TextStyle(color: RobotColors.secondaryText, fontSize: 24),
              )),
            ],
          ),
        ],
      ),
    );
  }
}
