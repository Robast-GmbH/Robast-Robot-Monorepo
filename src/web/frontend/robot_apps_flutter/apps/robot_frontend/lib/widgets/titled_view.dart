import 'package:flutter/material.dart';

class TitledView extends StatelessWidget {
  const TitledView({
    required this.title, required this.child, super.key,
    this.showBackButton = false,
  });
  final String title;
  final Widget child;
  final bool showBackButton;
  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        const SizedBox(height: 12),
        Stack(
          children: [
            if (showBackButton)
              Align(
                alignment: Alignment.centerLeft,
                child: Padding(
                  padding: const EdgeInsets.only(left: 8),
                  child: IconButton(
                    color: Colors.white,
                    iconSize: 32,
                    icon: const Icon(Icons.arrow_back),
                    onPressed: () {
                      Navigator.of(context).pop();
                    },
                  ),
                ),
              ),
            Align(
              child: Text(
                title,
                style: const TextStyle(
                  color: Colors.white,
                  fontSize: 32,
                ),
              ),
            ),
          ],
        ),
        const SizedBox(),
        Expanded(
          child: child,
        ),
      ],
    );
  }
}
