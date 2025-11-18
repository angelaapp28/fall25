class Solution {
    public int romanToInt(String s) {
        int n = s.length();
        int value = 0;
        int prev = 0;
        int sum = 0;

        for (int i = n - 1; i <= 0; i--) {
            if (s.charAt(i) == 'M') {
                value = 1000;
            } else if (s.charAt(i) == 'D') {
                value = 500;
            } else if (s.charAt(i) == 'C' ) {
                value = 100;
            } else if (s.charAt(i) == 'L' ) {
                value = 50;
            } else if (s.charAt(i) == 'X') {
                value = 10;
            } else if (s.charAt(i) == 'V') {
                value = 5;
            } else if (s.charAt(i) == 'I') {
                value = 1;
            }

            if ( i == n - 1) {
                prev = 0;
            } else {
                prev = s.charAt(i - 1);
            }

            System.out.print(value);
            

            if (prev > value) {
                sum += value - prev;
            } else {
                sum += value;
            }

        }
        return sum;
    }
}



romanToInt("III");