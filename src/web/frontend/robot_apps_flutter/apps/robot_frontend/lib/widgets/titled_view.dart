import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/status_bar.dart';

class TitledView extends StatelessWidget {
  const TitledView({
    required this.title,
    required this.child,
    super.key,
    this.showBackButton = false,
    this.onBackButtonPressed,
  });

  final String title;
  final Widget child;
  final bool showBackButton;
  final VoidCallback? onBackButtonPressed;

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        StatusBar(
          title: title,
          onBackButtonPressed: onBackButtonPressed ??
              () {
                Navigator.of(context).pop();
              },
        ),
        Expanded(
          child: child,
        ),
      ],
    );
  }
}
