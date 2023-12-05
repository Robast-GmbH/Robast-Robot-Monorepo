import 'package:flutter/material.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/pages/admin_page.dart';

class AdminMenuButton extends StatelessWidget {
  const AdminMenuButton({super.key});

  @override
  Widget build(BuildContext context) {
    return IconButton(
      onPressed: () {
        Navigator.push(
          context,
          MaterialPageRoute(
            builder: (context) => const AdminPage(),
          ),
        );
      },
      icon: const Icon(Icons.settings),
      color: AppColors.white,
      iconSize: 32,
    );
  }
}
