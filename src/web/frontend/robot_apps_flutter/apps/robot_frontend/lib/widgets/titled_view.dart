import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/keyboard_provider.dart';
import 'package:robot_frontend/widgets/status_bar.dart';

class TitledView extends StatelessWidget {
  const TitledView({
    required this.title,
    required this.child,
    super.key,
    this.showBackButton = false,
    this.collapsedTitle = false,
    this.onBackButtonPressed,
  });

  final String title;
  final Widget child;
  final bool showBackButton;
  final bool collapsedTitle;
  final VoidCallback? onBackButtonPressed;

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        if (!collapsedTitle) ...[
          StatusBar(
            title: title,
            showBackButton: showBackButton,
            onBackButtonPressed: onBackButtonPressed ??
                () {
                  Provider.of<KeyboardProvider>(context, listen: false).unfocus();
                  Navigator.of(context).pop();
                },
          ),
        ],
        Expanded(
          child: child,
        ),
      ],
    );
  }
}
