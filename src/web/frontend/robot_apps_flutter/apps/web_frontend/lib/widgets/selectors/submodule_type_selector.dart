import 'package:flutter/material.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:web_frontend/constants/web_colors.dart';
import 'package:web_frontend/widgets/rounded_container.dart';

class SubmoduleTypeSelector extends StatefulWidget {
  const SubmoduleTypeSelector({required this.controller, super.key});

  final SubmoduleSizeController controller;

  @override
  State<SubmoduleTypeSelector> createState() => _SubmoduleTypeSelectorState();
}

class _SubmoduleTypeSelectorState extends State<SubmoduleTypeSelector> {
  @override
  Widget build(BuildContext context) {
    return RoundedContainer(
        child: SizedBox(
      width: double.infinity,
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Benötigter Submodultyp',
              style: TextStyle(fontSize: 24, color: WebColors.secondaryText),
            ),
            const SizedBox(
              height: 8,
            ),
            GridView.count(
              crossAxisCount: 2,
              children: Submodule.sizesByDisplayName.entries.map((entry) {
                return SubmoduleSizeButton(
                    onPressed: () {
                      setState(() {
                        widget.controller.size = entry.key;
                      });
                    },
                    text: entry.value,
                    isSelected: widget.controller.size == entry.key,);
              }).toList(),
            ),
          ],
        ),
      ),
    ),);
  }
}

class SubmoduleSizeButton extends StatelessWidget {
  const SubmoduleSizeButton({required this.onPressed, required this.text, required this.isSelected, super.key});
  final VoidCallback onPressed;
  final String text;
  final bool isSelected;
  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: onPressed,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
        decoration: BoxDecoration(
          color: isSelected ? WebColors.accent : Colors.black.withOpacity(0.1),
          border: Border.all(color: Colors.white.withOpacity(0.4)),
          borderRadius: BorderRadius.circular(8),
        ),
        child: Text(
          text,
          textAlign: TextAlign.center,
          style: const TextStyle(
            fontSize: 24,
            color: WebColors.secondaryText,
          ),
        ),
      ),
    );
  }
}
