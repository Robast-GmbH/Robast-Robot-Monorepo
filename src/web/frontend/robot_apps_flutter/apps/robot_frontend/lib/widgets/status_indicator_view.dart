import 'package:flutter/material.dart';
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
        Icon(
          Icons.icecream,
          size: 48,
        ),
        SizedBox(width: 8),
        Icon(
          Icons.link,
          size: 48,
        ),
        SizedBox(width: 8),
        RotatedBox(
          quarterTurns: 1,
          child: Icon(
            Icons.battery_5_bar,
            size: 48,
          ),
        ),
        SizedBox(width: 8),
        DeveloperButtonView(
          onPressed: () {
            Navigator.push(context, MaterialPageRoute(builder: (context) => const ConfigPage()));
          },
          child: Container(
            margin: const EdgeInsets.all(4),
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: Color(0xFF00FF00),
              border: Border.all(
                color: Colors.white,
                width: 4,
              ),
            ),
            width: 38,
            height: 38,
          ),
        )
      ],
    );
  }
}
