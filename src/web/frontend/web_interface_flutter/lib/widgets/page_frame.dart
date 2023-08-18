import 'package:flutter/material.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';

class PageFrame extends StatelessWidget {
  const PageFrame({
    super.key,
    required this.title,
    required this.color,
    required this.child,
  });

  final String title;
  final Color color;
  final Widget child;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: AppColors.grey,
      body: Container(
        margin: const EdgeInsets.all(8),
        decoration: BoxDecoration(borderRadius: BorderRadius.circular(32), color: color),
        child: Column(
          children: [
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Padding(
                  padding: const EdgeInsets.only(top: 16, left: 16),
                  child: Text(
                    title,
                    style: const TextStyle(color: AppColors.white, fontSize: 20),
                  ),
                ),
                Padding(
                  padding: const EdgeInsets.only(right: 16, top: 16),
                  child: IconButton(
                    padding: const EdgeInsets.all(2),
                    onPressed: () {
                      Navigator.pop(context);
                    },
                    icon: const Icon(Icons.clear),
                    color: AppColors.white,
                    iconSize: 40,
                  ),
                )
              ],
            ),
            Expanded(child: child),
          ],
        ),
      ),
    );
  }
}
