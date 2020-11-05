#include <stdio.h>
#include <stdlib.h>

// Main () - execution starts here
int main (void)
{


    // Declare file stream variables
    FILE *fInput, *fOutput;

    // Other variables needed
    int i,d,D;
    char name[100];

    printf("care to enter a name for a file?");
    fgets(name,100, stdin);

    //---WRITING---

    // Try and open the text "sample.txt" (in the current directory) file for writing
    fOutput = fopen ("name", "w");

    // Check we were able to open the file
    if ( fOutput == NULL)
    {
        printf ("\nthe file could not be opened for writing, exiting");
        return -1;
    }

    // Use a loop to write values to the newly created file
    for ( i = 1 ; i <= 10 ; i++)
    {
        int s = i*i;
        fprintf (fOutput, "%d\t%d\n", i, s);
    }

    // And close the file
    fclose (fOutput);



    //---READING----

    // Try and open the text file "numbers " (in the current directory) file for reading
    fInput = fopen ("name", "r");

    // Check we were able to open the file
    if ( fInput == NULL)
    {
        printf ("\nthe file could not be opened for reading, exiting");
        return -1;
    }

    // Read, line by line the 10 values written into variable d
    // and then display the contents of d on the screen
    for ( i = 1 ; i <= 10 ; i++)
    {
        fscanf (fInput, "%d \t %d", &d, &D);
        printf ("Value read from file %d*%d = %d\n",d,d,D);
    }

    // And close the file
    fclose (fInput);

    return (0);     // Exit indicating success
}
