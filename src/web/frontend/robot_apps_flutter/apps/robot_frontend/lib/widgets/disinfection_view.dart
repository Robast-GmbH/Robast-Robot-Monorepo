import 'package:flutter/material.dart';
import 'package:robot_frontend/pages/menu_page.dart';

class DisinfectionView extends StatelessWidget {
  const DisinfectionView({required this.onTimeout, super.key});

  final VoidCallback onTimeout;

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        Expanded(
          child: GestureDetector(
            onTap: () => Navigator.push(
              context,
              MaterialPageRoute<MenuPage>(
                builder: (context) => const MenuPage(),
              ),
            ),
            child: const ColoredBox(
              color: Colors.transparent,
              child: Center(
                child: Text(
                  'Bitte Hände desinfizieren',
                  style: TextStyle(fontSize: 80),
                ),
              ),
            ),
          ),
        ),
      ],
    );
  }
}
