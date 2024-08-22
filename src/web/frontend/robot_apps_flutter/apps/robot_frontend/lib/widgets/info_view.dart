import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';

class InfoView extends StatelessWidget {
  const InfoView({super.key});

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Expanded(
          child: Column(
            children: [
              Expanded(
                child: CustomButtonView(
                  text: 'FAQ',
                  onPressed: () {},
                ),
              ),
              const SizedBox(height: 16),
              Expanded(
                child: CustomButtonView(
                  text: 'Datenschutz',
                  onPressed: () {},
                ),
              ),
            ],
          ),
        ),
        const SizedBox(width: 16),
        Expanded(
          child: Column(
            children: [
              Expanded(
                child: CustomButtonView(
                  text: 'Feedback',
                  onPressed: () {},
                ),
              ),
              const SizedBox(height: 16),
              Expanded(
                child: CustomButtonView(
                  text: 'Krankenhaus Info',
                  onPressed: () {},
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }
}
